#pragma once
#include "states/states.h"

void initDisplay();

void requestDisplayUpdate();
void startDisplayUpdates();
void stopDisplayUpdates();
bool handleDisplayUpdate(ion_state * state);
