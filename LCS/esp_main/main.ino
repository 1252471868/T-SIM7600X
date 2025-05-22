#include "global.h" // Needed for constants, externs (e.g., ESP_BT, LED_PIN)
#include "communication.h"
#include "sd_card.h"
#include "blynk_interface.h"
#include "modem_setup.h"
#include "system_utils.h"
#ifdef ENABLE_PUMP_CONTROL
#include "pump_control.h"
#endif // ENABLE_PUMP_CONTROL


// #define DUMP_AT_COMMANDS // Uncomment for detailed AT command debugging


// Sensor Objects (Example - Add BME680 init if needed)
// Adafruit_BME680 bme(&Wire); // BME680 sensor object (if used)

// Timers and Storage Objects

// System State Flags


/**
 * @brief Main setup function, runs once on boot.
 * Initializes serial communication, watchdog, LED, Arduino communication,
 * modem, network time, SD card, Blynk connection, and timers.
 */
void setup() {
    // Initialize serial console for debugging
    Serial.begin(UART_BAUD);
    delay(1000); // Wait for serial monitor connection
    Serial.println("\n--- ESP32 Environmental Sensor System Starting (Refactored) ---");

    // Initialize watchdog
    initializeWatchdog();
    // Initialize Bluetooth Serial for Pump Control
#ifdef ENABLE_PUMP_CONTROL
    setupPumpControl();
    resetWatchdog();
#endif // ENABLE_PUMP_CONTROL


    // Initialize indicator LED (set high initially)
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Initialize UART for Arduino communication
    Serial.println("Initializing UART for Arduino communication...");
    ArduinoSerial.begin(115200, SERIAL_8N1, ARDUINO_UART_RX, ARDUINO_UART_TX);
    delay(500); // Allow UART to settle
    resetWatchdog();

    // Try to verify communication with Arduino
    bool isArduinoCommOk = verifyArduinoCommunication(ArduinoSerial);
    if (!isArduinoCommOk) {
        Serial.println("Warning: Initial Arduino communication failed. System will continue, but data may be unavailable.");
        // Keep LED solid HIGH from init, or use a specific pattern
    }
    resetWatchdog();

    // Initialize modem
    setupModem();
    resetWatchdog();

    // Get and set time from network
    bool timeSet = setupTimeWithRetry();
    if (!timeSet) {
        Serial.println("Warning: Failed to set system time. SD filenames will use fallback format.");
    }
    resetWatchdog();

    // Setup SD card
    setupSD();
    resetWatchdog();

    // Initialize Blynk connection
    bool blynkInitialized = initializeBlynk(); // Uses function from blynk_interface.cpp
    resetWatchdog();

    resetWatchdog();
    Serial.println("--- Setup Complete --- ");
    digitalWrite(LED_PIN, LOW); // Turn LED off to indicate setup complete and running
}

/**
 * @brief Main loop function, runs repeatedly.
 * Handles Blynk communication, timer events, processes incoming Arduino commands,
 * processes incoming Pump Control commands (via Bluetooth), and resets the watchdog timer.
 */
void loop() {
    // Reset watchdog timer at the start of each loop iteration
    resetWatchdog();

    // Run Blynk tasks (connection handling, virtual pin processing)
    runBlynk(); // From blynk_interface.cpp

    // Process incoming commands from Arduino (UART)
    processIncomingCommands(ArduinoSerial); // Simplified call

    // Process incoming commands from second ESP32 (Bluetooth for Pump Control)
#ifdef ENABLE_PUMP_CONTROL
    if (ESP_BT.hasClient())
    {                                    // Check if a Bluetooth client is connected
        processIncomingCommands(ESP_BT); // Use the common command processor
    }
#endif // ENABLE_PUMP_CONTROL
    
} 