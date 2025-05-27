#ifndef BLUETOOTH_COMM_H
#define BLUETOOTH_COMM_H

#include <Arduino.h>

// Function declarations
bool initializeBluetoothComm();
void requestVOCData();
void processMainESP32Commands();
bool verifyMainESP32Connection();
float getLatestVOCReading();

#endif // BLUETOOTH_COMM_H 