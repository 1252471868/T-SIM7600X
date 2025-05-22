#ifndef SD_CARD_H
#define SD_CARD_H

#include <Arduino.h>
#include <SD.h>
#include <TimeLib.h> // Needed for timestamping filename and logs
#include <SPI.h>

// Extern declarations for sensor data variables (defined elsewhere, e.g., communication.cpp)

// Function Declarations
String createDataFilename();
void setupSD();
void reconnectSD();
void logToSD();

#endif // SD_CARD_H 