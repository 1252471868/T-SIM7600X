#include "system_utils.h"
#include "global.h"
#include <esp_task_wdt.h>

/**
 * @brief Initializes the watchdog timer
 */
void initializeWatchdog() {
    Serial.println("Initializing watchdog timer...");
    esp_task_wdt_init(WDT_TIMEOUT / 1000, true); // timeout in seconds, panic=true
    esp_task_wdt_add(NULL); // Add current task to watchdog
    Serial.println("Watchdog timer initialized");
}

/**
 * @brief Resets the watchdog timer to prevent system reset
 */
void resetWatchdog() {
    esp_task_wdt_reset();
}

/**
 * @brief Performs a system reset
 */
void resetSystem() {
    Serial.println("System reset requested. Restarting in 3 seconds...");
    
    // Stop all pump operations first
    // stopPump(); // Assuming this function exists
    
    delay(3000);
    ESP.restart();
} 