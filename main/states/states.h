#pragma once

#include <sys/unistd.h>

enum control_state { IDLE, CHARGING, START_CALIBRATE, TURN_MOTOR_ON, MOTOR_ON, SET_ASSIST_LEVEL, TURN_MOTOR_OFF, MOTOR_OFF };

struct ion_state {
    // The state we're in
    control_state state;

    // The step of the state we're in, most states follow a sequence of commands
    uint8_t step;

    // Should the display be 'on' ('off' is logo for CU3).
    bool displayOn;

    // Is assist currently on
    bool assistOn;

    // The assist level currently active (set in the motor)
    uint8_t levelSet;

    // Whether we should do (regular) handoffs to other components
    bool doHandoffs;

    // Motor indicates off is ready.
    bool motorOffAck;

    // Current assist level
    uint8_t level;

    // Speed in km/h * 10
    uint16_t speed;
};

void toIdleState(ion_state * state);
void handleIdleState(ion_state * state, bool modeShortPress);

void toTurnMotorOnState(ion_state * state);
void handleTurnMotorOnState(ion_state * state);

void toMotorOnState(ion_state * state);
void handleMotorOnState(ion_state * state, bool modeShortPress, bool lightLongPress, bool calibrate);

void toChargingState(ion_state * state);
void handleChargingState(ion_state * state, bool chargePin);

void toCalibrateState(ion_state * state);
void handleCalibrateState(ion_state * state);

void handleSetAssistLevelState(ion_state * state);
void toSetAssistLevelState(ion_state * state);

void toTurnMotorOffState(ion_state * state);
void handleTurnMotorOffState(ion_state * state);

void toMotorOffState(ion_state * state);
void handleMotorOffState(ion_state * state, bool modeShortPress, bool wakeup);
