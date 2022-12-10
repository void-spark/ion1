#include "states/states.h"
#include "trip.h"
#include "relays.h"
#include "bat.h"
#include "cu2.h"
#include "cu3.h"
#include "display.h"

static EventGroupHandle_t eventGroupHandle;

#if CONFIG_ION_CU2 || CONFIG_ION_CU3
static const int DISPLAY_UPDATE_BIT = BIT0;
#endif

void initDisplay() {
    eventGroupHandle = xEventGroupCreate();
}

void requestDisplayUpdate() {
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
    xEventGroupSetBits(eventGroupHandle, DISPLAY_UPDATE_BIT);
#endif
}

static void displayUpdate(ion_state * state) {
#if CONFIG_ION_CU2
    uint16_t numTop = digits( state->speed, 3, 2);
    uint32_t numBottom = digits(getTrip1() / 100, 5, 1);
    displayUpdateCu2(false, (assist_level)state->level, BLNK_SOLID, BLNK_OFF, BLNK_OFF, BLNK_SOLID, getLight() ? BLNK_SOLID : BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_SOLID, BLNK_SOLID,
                  false, getBatPercentage(), numTop, numBottom);
#elif CONFIG_ION_CU3
    displayUpdateCu3(DSP_SCREEN, state->displayOn, getLight(), false, state->level, state->speed, getTrip1(), getTrip2());
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
        return true;
    }

#endif
    return false;
}
