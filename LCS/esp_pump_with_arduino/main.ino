#include "global.h"
#include "blynk_interface.h"
#include "modem_setup.h"
#include "pump_control.h"
#include "system_utils.h"
#include "communication.h"

/**
 * @brief Main setup function, runs once on boot.
 * Initializes serial communication, watchdog, LED, Arduino communication,
 * modem, network time, Blynk connection, and pump systems.
 */
void setup() {
    // Initialize serial console for debugging
    Serial.begin(UART_BAUD);
    delay(2000); // Increased wait for serial monitor connection and system stability
    Serial.println("\n--- ESP32 Pump Controller Starting ---");
    
    // Check initial memory status
    Serial.print("Initial free heap: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Minimum free heap: ");
    Serial.println(ESP.getMinFreeHeap());

    // Initialize watchdog with longer timeout
    initializeWatchdog();
    resetWatchdog();

    // Initialize indicator LED (set high initially)
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    resetWatchdog();

    // Initialize UART for Arduino communication first (lightweight)
    Serial.println("Initializing UART for Arduino communication...");
    ArduinoSerial.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_UART_RX, ARDUINO_UART_TX);
    delay(500); // Allow UART to settle
    resetWatchdog();

    // Try to verify communication with Arduino
    bool isArduinoCommOk = verifyArduinoConnection();
    if (!isArduinoCommOk) {
        Serial.println("Warning: Initial Arduino communication failed. System will continue, but pump control may be unavailable.");
    }
    resetWatchdog();

    // Initialize modem (heavy operation)
    Serial.println("Starting modem initialization...");
    setupModem();
    resetWatchdog();

    // Get and set time from network
    bool timeSet = setupTimeWithRetry();
    if (!timeSet)
    {
      Serial.println("Warning: Failed to set system time. Timestamps will use fallback format.");
    }
    resetWatchdog();

    // Initialize Blynk connection (after modem is stable)
    Serial.println("Starting Blynk initialization...");
    bool blynkInitialized = initializeBlynk();
    resetWatchdog();

    // Initialize pump systems
    Serial.println("Initializing pump control systems...");
    pumpOp.mode = PUMP_OFF;
    pumpOp.enabled = false;
    resetWatchdog();
    
    Serial.println("--- Setup Complete ---");
    Serial.println("VOC data will be received via Blynk API from main ESP32");
    digitalWrite(LED_PIN, LOW); // Turn LED off to indicate setup complete and running
}

/**
 * @brief Main loop function, runs repeatedly.
 * Handles Blynk communication, timer events, pump operations, and watchdog reset.
 */
void loop() {
    // Reset watchdog timer at the start of each loop iteration
    resetWatchdog();

    // Process incoming commands from Blynk/main ESP32
    processIncomingCommands(Serial);

    // Run Blynk tasks (connection handling, virtual pin processing)
    runBlynk();

    // Process pump operations and safety checks
    processPumpOperations();

    // Small delay to prevent overwhelming the system
    delay(10);
}
