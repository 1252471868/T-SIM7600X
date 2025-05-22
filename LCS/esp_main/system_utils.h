#ifndef SYSTEM_UTILS_H
#define SYSTEM_UTILS_H

#include <Arduino.h>
#include <esp_task_wdt.h> // For watchdog functions

// Function Declarations
void resetSystem();
void initializeWatchdog();
void resetWatchdog();

#endif // SYSTEM_UTILS_H 