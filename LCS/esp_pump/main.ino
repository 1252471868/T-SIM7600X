#include "global.h"
#include "blynk_interface.h"
#include "modem_setup.h"
#include "pump_control.h"
#include "system_utils.h"
#include "bluetooth_comm.h"

/**
 * @brief Main setup function, runs once on boot.
 * Initializes serial communication, watchdog, LED, Arduino communication,
 * modem, network time, Blynk connection, and pump systems.
 */
void setup() {
    // Initialize serial console for debugging
    Serial.begin(UART_BAUD);
    delay(1000); // Wait for serial monitor connection
    Serial.println("\n--- ESP32 Pump Controller Starting ---");

    // Initialize watchdog
    initializeWatchdog();
    resetWatchdog();

    // Initialize indicator LED (set high initially)
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Initialize Bluetooth communication with main ESP32
    Serial.println("Initializing Bluetooth communication...");
    bool bluetoothInitialized = initializeBluetoothComm();
    if (!bluetoothInitialized) {
        Serial.println("Warning: Bluetooth initialization failed. VOC auto-trigger may not work.");
    }
    resetWatchdog();

    // Initialize UART for Arduino communication
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

    // Initialize modem
    setupModem();
    resetWatchdog();


    resetWatchdog();

    // Initialize Blynk connection
    bool blynkInitialized = initializeBlynk();
    resetWatchdog();
    // Get and set time from network
    bool timeSet = setupTimeWithRetry();
    if (!timeSet)
    {
      Serial.println("Warning: Failed to set system time. Timestamps will use fallback format.");
    }
    // Initialize pump systems
    Serial.println("Initializing pump control systems...");
    pumpOp.mode = PUMP_OFF;
    pumpOp.enabled = false;
    
    Serial.println("--- Setup Complete ---");
    digitalWrite(LED_PIN, LOW); // Turn LED off to indicate setup complete and running
}

/**
 * @brief Main loop function, runs repeatedly.
 * Handles Blynk communication, timer events, pump operations, and watchdog reset.
 */
void loop() {
    // Reset watchdog timer at the start of each loop iteration
    resetWatchdog();

    // Run Blynk tasks (connection handling, virtual pin processing)
    runBlynk();

    // Process Bluetooth communication with main ESP32
    processMainESP32Commands();

    // Process pump operations and safety checks
    processPumpOperations();

    // Small delay to prevent overwhelming the system
    delay(10);
}
