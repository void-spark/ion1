#include "ctrl_event_group.h"

static EventGroupHandle_t controlEventGroup;

void initControlEventGroup() {
    controlEventGroup = xEventGroupCreate();
}

void setControlBits(const EventBits_t uxBitsToSet) {
    xEventGroupSetBits(controlEventGroup, uxBitsToSet);
}

void clearControlBits(const EventBits_t uxBitsToClear) {
    xEventGroupClearBits(controlEventGroup, uxBitsToClear);
}

EventBits_t waitControlBits(const EventBits_t uxBitsToWaitFor,
                            const BaseType_t xClearOnExit,
                            const BaseType_t xWaitForAllBits,
                            TickType_t xTicksToWait) {
    return xEventGroupWaitBits(controlEventGroup, uxBitsToWaitFor, xClearOnExit, xWaitForAllBits, xTicksToWait);
}