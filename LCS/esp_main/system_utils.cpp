#include "system_utils.h"
#include "global.h"

// Globals are accessed via extern declarations in globals.h

/**
 * @brief Performs a software reset of the ESP32.
 * Logs a message and then calls ESP.restart().
 */
void resetSystem() {
    Serial.println("Performing ESP32 system reset...");
    delay(500); // Short delay to allow logging
    // It's good practice to deinitialize WDT before reset if possible,
    // though ESP.restart() should handle it.
    esp_task_wdt_deinit(); 
    ESP.restart();
}

/**
 * @brief Initializes the ESP Task Watchdog Timer.
 */
void initializeWatchdog() {
    Serial.println("Initializing watchdog timer...");
    esp_task_wdt_init(WDT_TIMEOUT, true); // Use timeout from header, panic on timeout
    esp_task_wdt_add(NULL);     // Add current task (usually setup/loop) to watchdog
    resetWatchdog(); // Initial reset
}

/**
 * @brief Resets the ESP Task Watchdog Timer.
 */
void resetWatchdog() {
    esp_task_wdt_reset();
} 