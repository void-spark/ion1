#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "states/states.h"
#include "trip.h"
#include "relays.h"
#include "bat.h"
#include "cu2.h"
#include "cu3.h"
#include "display.h"

static EventGroupHandle_t eventGroupHandle;
static TimerHandle_t displayUpdateTimer;

#if CONFIG_ION_CU2 || CONFIG_ION_CU3
static const int DISPLAY_UPDATE_BIT = BIT0;
#endif


static void displayUpdateTimerCallback(TimerHandle_t xTimer) { requestDisplayUpdate(); }

void initDisplay() {
    eventGroupHandle = xEventGroupCreate();
    displayUpdateTimer = xTimerCreate("displayUpdateTimer", (1500 / portTICK_PERIOD_MS), pdTRUE, (void *)0, displayUpdateTimerCallback);
}

void requestDisplayUpdate() {
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
    xEventGroupSetBits(eventGroupHandle, DISPLAY_UPDATE_BIT);
#endif
}

void startDisplayUpdates() {
    xTimerStart(displayUpdateTimer, 0);
}

void stopDisplayUpdates() {
    xTimerStop(displayUpdateTimer, 0);
}

static void displayUpdate(ion_state * state) {
#if CONFIG_ION_CU2
    uint16_t numTop = digits(state->speed, 3, 2);
    uint32_t numBottom = digits(getTrip1() / 100, 5, 1);
    displayUpdateCu2(false, // setDefault
                     (assist_level)state->level, // assistLevel
                     BLNK_SOLID, // assistBlink
                     BLNK_OFF, // wrench
                     BLNK_OFF, // total
                     BLNK_SOLID, // trip
                     getLight() ? BLNK_SOLID : BLNK_OFF, // light
                     state->state == CHARGING ? BLNK_SLOW : BLNK_SOLID, // bars
                     BLNK_OFF, // comma
                     BLNK_SOLID, // km
                     BLNK_SOLID, // top
                     BLNK_SOLID, // bottom
                     false, // miles
                     getBatPercentage(), // batPercentage
                     numTop, // topVal
                     numBottom); // bottomVal
#elif CONFIG_ION_CU3
    displayUpdateCu3(state->state == CHARGING ? DSP_BAT_CHARGE : DSP_SCREEN, state->displayOn, getLight(), false, state->level, state->speed, getTrip1(), getTrip2());
#endif
}

bool handleDisplayUpdate(ion_state * state) {
#if CONFIG_ION_CU2
    if(cu2HandleDisplayUpdate()) {
        return true;
    }
#endif

#if CONFIG_ION_CU2 || CONFIG_ION_CU3
    EventBits_t bitsToCheck =  DISPLAY_UPDATE_BIT;

    EventBits_t bits = xEventGroupWaitBits(eventGroupHandle, bitsToCheck, false, false, 0);
    if((bits & DISPLAY_UPDATE_BIT) != 0) {
        xEventGroupClearBits(eventGroupHandle, DISPLAY_UPDATE_BIT);
        displayUpdate(state);
        xTimerReset(displayUpdateTimer, 0);
        return true;
    }

#endif
    return false;
}
