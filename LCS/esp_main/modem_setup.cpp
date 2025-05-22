
#include "modem_setup.h"
#include "global.h"

/**
 * @brief Initializes the cellular modem (SIM7600).
 * Sets up UART communication, toggles power and flight mode pins,
 * attempts to restart the modem up to 5 times, and logs the modem name.
 */
void setupModem() {
    Serial.println("Initializing modem...");
    // Begin UART communication with the modem
    // SerialAT needs to be defined globally (usually as a HardwareSerial instance)
    extern HardwareSerial SerialAT; 
    SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(100);

    // Power cycle the modem using PWRKEY pin
    Serial.println("Toggling modem power key...");
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW);  // Ensure it's low initially
    delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH); // Pull high briefly
    delay(300);
    digitalWrite(MODEM_PWRKEY, LOW);  // Back to low (normal state)

    // Ensure modem is not in flight mode
    pinMode(MODEM_FLIGHT, OUTPUT);
    digitalWrite(MODEM_FLIGHT, HIGH); // High usually means flight mode OFF
    delay(1000); // Give modem time to respond to pin changes

    Serial.println("Attempting modem restart (up to 5 times)...");
    bool modemRestarted = false;
    for (int attempt = 1; attempt <= 5; ++attempt) {
        Serial.print("Modem restart attempt "); Serial.println(attempt);
        if (modem.restart()) {
            Serial.println("Modem restarted successfully.");
            modemRestarted = true;
            break; // Exit loop on success
        } else {
            Serial.println("Modem restart attempt failed.");
            if (attempt < 5) {
                Serial.println("Waiting 2 seconds before next attempt...");
                delay(2000); // Wait before retrying
            }
        }
    }

    if (!modemRestarted) {
        Serial.println("Warning: Failed to restart modem via AT command after 5 attempts. Proceeding anyway...");
    }

    String name = modem.getModemName();
    Serial.println("Modem Name: " + name);
}

/**
 * @brief Attempts to get the current time from the cellular network and set the system time.
 * Retries several times with delays if the initial attempts fail.
 *
 * @return true If the network time was successfully obtained and the system time set.
 * @return false If time could not be obtained after multiple retries.
 */
bool setupTimeWithRetry() {
    Serial.println("Attempting to set system time from network...");
    int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
    float timezone = 0;
    const int maxTimeRetries = 5;

    for (int attempt = 1; attempt <= maxTimeRetries; attempt++) {
        Serial.print("Requesting network time (Attempt "); Serial.print(attempt); Serial.println(")...");
        if (modem.getNetworkTime(&year, &month, &day, &hour, &min, &sec, &timezone)) {
            // Basic validation of the received time
            if (year > 2020) { 
                Serial.printf("Network time received: %d-%02d-%02d %02d:%02d:%02d TZ:%.1f\n", 
                              year, month, day, hour, min, sec, timezone);
                // Set the ESP32 system time using TimeLib's setTime function
                setTime(hour, min, sec, day, month, year);
                // Optional: Adjust for timezone if needed, though TimeLib doesn't directly use it.
                Serial.println("System time set successfully.");
                return true;
            } else {
                Serial.println("Received invalid year from network time. Retrying...");
            }
        } else {
            Serial.println("Failed to get network time.");
        }
        // Wait before retrying
        if (attempt < maxTimeRetries) {
            Serial.println("Retrying in 10 seconds...");
            delay(10000);
        }
    }

    Serial.println("Failed to set system time from network after multiple attempts.");
    return false;
} 