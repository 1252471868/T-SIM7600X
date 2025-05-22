#ifndef MODEM_SETUP_H
#define MODEM_SETUP_H

#include <Arduino.h>

#include <TimeLib.h>      // Required for time setting function

// Function Declarations
void setupModem();
bool setupTimeWithRetry();

#endif // MODEM_SETUP_H 