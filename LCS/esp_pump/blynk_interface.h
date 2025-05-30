#ifndef BLYNK_INTERFACE_H
#define BLYNK_INTERFACE_H

#include "global.h"

// Function declarations
bool initializeBlynk();
void runBlynk();
void updateBlynkStatus();
void sendPumpDataToBlynk();
void checkVOCLevels();
void checkTimedOperations();

// Function to update Blynk pump status (called from pump_control.cpp)
void updateBlynkPumpStatus(bool enabled);

#endif // BLYNK_INTERFACE_H 