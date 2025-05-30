#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include <Arduino.h>

// Function declarations
bool sendCommandToArduino(const char* cmd, const String& data);
bool verifyArduinoConnection();
void startPump(bool inflate, int flowRate);
void stopPump();
void updatePumpFlowRate(int flowRate);
void startSamplingMode();
void emergencyStop();
void processPumpOperations();

#endif // PUMP_CONTROL_H 