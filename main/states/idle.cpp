#include "esp_log.h"
#include "bow.h"
#include "cu2.h"
#include "states.h"

static const char *TAG = "idle_state";

/**
 * Idle state, this is what we start at, or go to if the motor no longer responds.
 * We wait for a esp32 button click, or a wakeup message/byte '0x00' on the bus.
 * On either we switch to turn motor on state.
 * We continuously read the bus, waiting for a wakeup message/byte.
 * The motor relay should (already) be off in this state.
*/
void toIdleState(ion_state * state) {
    state->state = IDLE;
    state->step = 0;
}

void handleIdleState(ion_state * state, bool modeShortPress) {
    if(modeShortPress) {
        // We are not polling CU2 yet, so this would be from the ESP32 button.
        toTurnMotorOnState(state);
        return;
    }

    messageType message = {};
    readResult result = readMessage(&message, 50 / portTICK_PERIOD_MS );

    if(result == MSG_WAKEUP) {
        // Received a '0x00' byte, sent when connecting a display, or pressing a button while the display is 'sleeping'.        
        ESP_LOGI(TAG, "Wakeup!");
#if CONFIG_ION_CU2
        // If the '0x00' byte is from pressing a CU2 button, we don't want to handle it again as a button press.
        ignorePress();
#endif
        toTurnMotorOnState(state);
        return;
    }

    if(result == MSG_OK) {
        // TODO: Maybe wake on certain bus messages, display might be awake if the esp32 reset.
        ESP_LOGI(TAG, "Incoming: Tgt:%d, Src:%d, Type:%d, Command:%d", message.target, message.source, message.type, message.command);
        ESP_LOG_BUFFER_HEX(TAG, message.payload, message.payloadSize);
        return;
    }
}
