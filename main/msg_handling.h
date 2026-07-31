#pragma once

#include "states/states.h"

enum messageHandlingResult {
    // We got a handoff back, so we get to send the next message
    CONTROL_TO_US,
    // We had to reply to a message, so sender is next to send a message.
    CONTROL_TO_SENDER,
    // We did not get a timely reply to a handoff message.
    HANDOFF_TIMEOUT
};

messageHandlingResult handleMessage(ion_state * state);