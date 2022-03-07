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

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

#define RX_BUF_SIZE (1024)

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

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_intr_config(UART_NUM_2, &uart_intr));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

messageType readMessage(TickType_t timeout) {

    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE);

    bool escaping = false;

    messageType message = {};
    message.target = -1;
    message.source = -1;
    message.type = -1;
    message.size = -1;

    while(true) {
        size_t rxReady = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, &rxReady));
        if(rxReady == 0) {
            // Always wait for at least one byte, even if there is none available.
            rxReady = 1;
        }

        const int rxBytes = uart_read_bytes(UART_NUM_2, data, rxReady, timeout > 0 ? timeout : 1000 / portTICK_PERIOD_MS);
        if(timeout > 0 && rxBytes == 0) {
            message.timeout = true;
            free(data);
            return message;
        }

        for(int bufferPos = 0; bufferPos < rxBytes; bufferPos++) {

            uint8_t byteRead = data[bufferPos];
            ESP_LOGD(TAG, "R:%02x", byteRead);

            uint8_t input[2] = {};
            uint8_t inputLen = 0;

            if(escaping) {
                input[0] = 0x10;
                inputLen += 1;
                if(byteRead != 0x10) {
                    input[1] = byteRead;
                    inputLen += 1;
                    if(message.length != 0) {
                        // Unescaped 0x10, reset
                        if(inputLen > 0) {
                            ESP_LOGI(TAG, "Incomplete:");
                            ESP_LOG_BUFFER_HEX(TAG, message.data, message.length);
                        }
                        message.length = 0;
                        message.target = -1;
                        message.source = -1;
                        message.type = -1;
                        message.size = -1;
                    }
                }
                escaping = false;
            } else if(byteRead == 0x10) {
                escaping = true;
                // Message start byte, we need to check the next input byte to decide what to do.
                continue;
            } else {
                // Not a message start byte, and we're not escaping, so just parse it normally.
                input[0] = byteRead;
                inputLen += 1;
            }

            for(int pos = 0; pos < inputLen; pos++) {
                uint8_t value = input[pos];
                uint8_t low = value & 0x0f;
                uint8_t high = value >> 4;

                if(message.length == 0) {
                    // First byte in a new message
                    if(value == 0x00) {
                        // Single '00' with no leading '10', which is sent by display to wake up system.
                        message.wakeup = true;
                        free(data);
                        return message;
                    }
                } else if(message.length == 1) {
                    // First nibble is always message target.
                    message.target = high;
                    // Second nibble is always message type.
                    message.type = low;
                } else if(message.length == 2) {
                    if(message.type == 0x00) {
                        message.size = 3;
                    } else {
                        message.source = high;
                        if(message.type == 0x03 || message.type == 0x04) {
                            message.size = 4;
                        } else {
                            message.size = low + 5;
                        }
                    }
                }
                message.data[message.length++] = value;

                if(message.length > 2 && message.length == message.size) {
                    free(data);

                    // ESP_LOGI(TAG, "Tgt:%d, Src:%d, Type:%d, Size:%d", message.target, message.source, message.type, message.size);

                    // uint8_t crc = crc8_bow( message.data, message.length - 1);

                    // ESP_LOGI(TAG, "CRC(calc): %02x", crc);

                    return message;
                }
            }
        }
    }
}

messageType readMessage() {
    return readMessage(0);
}

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
    uart_write_bytes(UART_NUM_2, escaped, outPos);
}

messageType exchange(uint8_t *cmd, size_t cmdLen, const TickType_t timeout) {
    messageType message;

    writeMessage(cmd, cmdLen);
    while(true) {
        message = readMessage(timeout);
        if(message.timeout) {
            writeMessage(cmd, cmdLen);
        } else if(!message.wakeup && message.target == 0x02) {
            break;
        }
    }

    if(message.data[3] != cmd[2]) { // Watch out, cmd doesn't include the leading 0x10
        ESP_LOGE(TAG, "Wrong reply cmd, expected %02x, got %02x", cmd[2], message.data[3]);
    }

    return message;
}

messageType exchange(uint8_t *cmd, size_t cmdLen) {
    return exchange(cmd, cmdLen, 0);
}
