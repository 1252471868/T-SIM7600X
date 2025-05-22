#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <Stream.h>
#include <ArduinoJson.h>

// Function Declarations
void processIncomingCommands(Stream& inputPort);
void sendCommandWithoutResponse(Stream& serialPort, const char* cmd, const String& data = "");
bool sendCommand(Stream& serialPort, const char* cmd, const String& data, unsigned long timeout);
bool sendCommandWithRetry(Stream& serialPort, const char* cmd, const String& data, int maxRetries);
bool verifyArduinoCommunication(Stream& serialPort);
bool checkNoInternetMode(Stream& serialPort);
void sendSensorDataCMD(Stream& serialPort);
void sendSensorData(); // Keep this separate for now, might move parts later

#endif // COMMUNICATION_H 