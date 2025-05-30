#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include <Arduino.h>

// Function declarations for optimized L298N pump control
void initializePumpControl();
bool verifyPumpHardware();
void setPumpControl(int pumpNumber, bool enabled, int dutyCycle);
void setValveControl(bool enabled);
void executeInflate(int dutyCycle);
void executeDeflate(int dutyCycle);
void stopAllPumpsAndValve();
void startPump(bool inflate, int flowRate);
void stopPump();
void updatePumpFlowRate(int flowRate);
void startVOCTriggeredSampling();
void startTimedSampling();
bool isWithinTimedWindow();
void emergencyStop();
void processPumpOperations();
void printPumpStatus();

#endif // PUMP_CONTROL_H 