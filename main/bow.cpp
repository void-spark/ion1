#include <string.h>
#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "esp_log.h"
#include "crc8.h"
#include "bow.h"

static const char *TAG = "bow";

#define UART_NUM (CONFIG_ION_UART)
#define TXD_PIN (CONFIG_ION_TXD)
#define RXD_PIN (CONFIG_ION_RXD)

#define RX_BUF_SIZE (1024)

struct parserState {
    // Are we holding the last byte to check if it's escaped.
    bool escaping;

    // We found a unescaped 0x10, indicating start of message.
    bool started;

    // Payload length is indicated by one nibble, so max value 0xF (15).
    // Payload length excludes the starting byte, 2 header bytes, command byte, and crc byte.
    // So total length = payload + 5, and max length is 15 + 5 = 20.
    uint8_t data[20];
    uint8_t length; // readPos?

    // Header values.
    uint8_t target;
    uint8_t source;
    int8_t type;
    int8_t size;
};

static uint8_t nibbles(uint8_t left, uint8_t right) {
    return (uint8_t) (right | (left << 4));
}

void initUart() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    uart_intr_config_t uart_intr = {};
    uart_intr.intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M | UART_BRK_DET_INT_ENA_M | UART_PARITY_ERR_INT_ENA_M;

    uart_intr.rxfifo_full_thresh = 1; // This should speed things up.
    uart_intr.rx_timeout_thresh = 10;
    uart_intr.txfifo_empty_intr_thresh = 10;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_intr_config(UART_NUM, &uart_intr));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

/**
 * @brief Parse a single message input byte.
 *
 * @param value the byte
 * @param message the current message state
 */
readResult parseByte(uint8_t value, parserState *state) {
    uint8_t low = value & 0x0f;
    uint8_t high = value >> 4;

    if(state->length == 0) {
        // First byte in a new message, always 0x10.
        // We could check it, but that's already handled by the caller.
        // We still need it, to calculate the crc.
    } else if(state->length == 1) {
        // First nibble is always message target.
        state->target = high;
        // Second nibble is always message type.
        state->type = low;
    } else if(state->length == 2) {
        if(state->type == MSG_HANDOFF) {
            state->size = 3;
        } else {
            state->source = high;
            if(state->type == MSG_PING_RESP || state->type == MSG_PING_REQ) {
                state->size = 4;
            } else {
                state->size = low + 5;
            }
        }
    }

    state->data[state->length++] = value;

    if(state->length > 2 && state->length == state->size) {
        uint8_t crc = crc8_bow(state->data, state->length - 1);
        if(crc != state->data[state->length - 1]) {
            ESP_LOGI(TAG, "CRC error, message:");
            ESP_LOG_BUFFER_HEX(TAG, state->data, state->length);
            return MSG_CRC_ERROR;
        }
        return MSG_OK;
    }

    return MSG_CONTINUE;
}

readResult handleByte(uint8_t value, parserState *state) {
    if(state->started) {
        // Not a message start byte, and we're not escaping, so just parse it normally.
        return parseByte(value, state);
    } else if(value == 0x00) {
        // Single 0x00 with no leading 0x10, which is sent by display to wake up system.
        return MSG_WAKEUP;
    } else {
        // Unexpected bytes, continue till we find a 0x10 or 0x00
        return MSG_CONTINUE;
    }
}

/**
 * Deals with message framing, (re)starts a message on a unescaped 0x10,
 * and converts escaped 0x10s to single 0x10s
 */
readResult handleFraming(uint8_t value, parserState *state) {
    if(state->escaping) {
        state->escaping = false;
        if(value == 0x10) {
            // Escaped 0x10, don't reset and just parse the value.
            return handleByte(0x10, state);
        }

        // Non escaped 0x10, start of message.
        if(state->length != 0) {
            // We already were reading a message which we didn't get fully.
            // Ignore it and reset state.
            if(state->length > 0) {
                ESP_LOGI(TAG, "Incomplete message:");
                ESP_LOG_BUFFER_HEX(TAG, state->data, state->length);
            }
            *state = {};
        }

        state->started = true;
        // Record the start byte, no need to check result since it's always MSG_CONTINUE.
        handleByte(0x10, state);
        // First content byte of the message.
        return handleByte(value, state);
    } else if(value == 0x10) {
        state->escaping = true;
        // Message start byte, we need to check the next input byte to decide what to do.
        return MSG_CONTINUE;
    } else {
        return handleByte(value, state);
    }
}

readResult readMessage(messageType *message, TickType_t timeout) {
    uint8_t data[RX_BUF_SIZE];
    parserState state = {};

    while(true) {
        size_t rxReady = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, &rxReady));
        if(rxReady == 0) {
            // Always wait for at least one byte, even if there is none available.
            rxReady = 1;
        }

        const int rxBytes = uart_read_bytes(UART_NUM, data, rxReady, timeout > 0 ? timeout : 1000 / portTICK_PERIOD_MS);
        if(timeout > 0 && rxBytes == 0) {
            return MSG_TIMEOUT;
        }

        for(size_t bufferPos = 0; bufferPos < rxBytes; bufferPos++) {
            readResult result = handleFraming(data[bufferPos], &state);
            if(result != MSG_CONTINUE) {
                if(result == MSG_OK) {
                    message->target = state.target;
                    message->source = state.source;
                    message->type = state.type;
                    if(state.size >= 5) {
                        message->command = state.data[3];
                        memcpy(message->payload, state.data + 4, state.size - 5);
                        message->payloadSize = state.size - 5;
                    }
                }
                return result;
            }
        }
    }
}

readResult readMessage(messageType *message) { return readMessage(message, 0); }

void writeMessage(uint8_t *message, uint8_t messageLen) {

    // First create the full message, unescaped, includig crc.
    uint8_t data[20];
    data[0] = 0x10;
    memcpy(data + 1, message, messageLen);
    data[messageLen + 1] = crc8_bow(data, messageLen + 1);

    // Now create an escaped copy
    uint8_t escaped[20 * 2];
    uint8_t outPos = 0;
    escaped[outPos++] = 0x10;
    for(uint8_t inPos = 1; inPos < messageLen + 2; inPos++) {
        escaped[outPos++] = data[inPos];
        if(data[inPos] == 0x10) {
            escaped[outPos++] = 0x10;
        }
    }
    uart_write_bytes(UART_NUM, escaped, outPos);
}

void writeMessage(const messageType& message) {
    uint8_t data[18] = {};
    size_t length = 1;
    data[0] = nibbles(message.target, message.type);
    if(message.type == MSG_PING_REQ || message.type == MSG_PING_RESP) {
        data[1] = nibbles(message.source, 0);
        length = 2;
    } else if(message.type == MSG_CMD_REQ || message.type == MSG_CMD_RESP) {
        data[1] = nibbles(message.source, message.payloadSize);
        data[2] = message.command;
        memcpy(data + 3, message.payload, message.payloadSize);
        length = 3 + message.payloadSize;
    }
    writeMessage(data, length);
}

readResult exchange(const messageType& outMessage, messageType *inMessage, const TickType_t timeout) {
    writeMessage(outMessage);
    readResult result;
    while(true) {
        result = readMessage(inMessage, timeout);
        if(result == MSG_TIMEOUT) {
            // Retry
            writeMessage(outMessage);
        } else if(result == MSG_OK && inMessage->target == 0x02) {
            // We got our response
            break;
        }
    }

    if(inMessage->command != outMessage.command) {
        ESP_LOGE(TAG, "Wrong reply cmd, expected %02x, got %02x", outMessage.command, inMessage->command);
    }

    return result;
}

readResult exchange(const messageType& outMessage, messageType *inMessage) { 
    return exchange(outMessage, inMessage, 0); 
}

void exchange(const messageType& outMessage) {
    messageType response = {};
    readResult result = exchange(outMessage, &response);
}
