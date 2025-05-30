#ifndef BLYNK_INTERFACE_H
#define BLYNK_INTERFACE_H

#include <Arduino.h>

// Function declarations
bool initializeBlynk();
void runBlynk();
void updateBlynkStatus();
void sendPumpDataToBlynk();
void checkVOCLevels();
void checkTimedOperations();

#endif // BLYNK_INTERFACE_H 