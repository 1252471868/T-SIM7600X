
#include "global.h"
#include "blynk_interface.h"
#include <BlynkSimpleTinyGSM.h>
#include "sensors.h"
#include "sd_card.h"
#include "communication.h" // For sendCommand
#include "system_utils.h"  // For resetSystem
// Define global variables specific to Blynk Interface

char auth[] = "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI"; // Assuming BLYNK_AUTH_TOKEN is in EnvSensor.h
char apn[]  = ""; // Defined here now
char user[] = ""; // Defined here now
char pass[] = ""; // Defined here now
BlynkTimer timer; // Defined here now



/**
 * @brief Initializes the Blynk connection.
 * Attempts to connect to GPRS and then the Blynk server.
 * @return true if successfully connected to Blynk, false otherwise.
 */
bool initializeBlynk() { // Renamed from setupBlynk to avoid confusion with main setup()
    bool blynkInitialized = false;
    Serial.println("Initializing Blynk connection...");
    Blynk.config(modem, auth, BLYNK_DEFAULT_DOMAIN, BLYNK_DEFAULT_PORT);
    Serial.println("Connecting to GPRS network...");
    if(Blynk.connectNetwork(apn, user, pass)) {
        Serial.println("GPRS network connected. Connecting to Blynk...");
        if (Blynk.connect(BLYNK_CONNECT_TIMEOUT)) { // Timeout for Blynk connection
            blynkConnected = true; // Set flag only on successful connect
            internetAvailable = true;
            Serial.println("Successfully connected to Blynk!");
            // Notify Arduino (assuming ArduinoSerial is the target)
            sendCommand(ArduinoSerial, CMD_CONNECTED, "", CMD_TIMEOUT); 
            blynkInitialized = true;
        } else {
            Serial.println("Failed to connect to Blynk server.");
            blynkConnected = false;
            internetAvailable = false;
            blynkInitialized = false;
        }
    } else {
        Serial.println("Failed to connect to GPRS network.");
        blynkConnected = false;
        internetAvailable = false;
        blynkInitialized = false;
    }

    // Fallback logic if Blynk connection failed
    if (!blynkInitialized || !internetAvailable)
    {
        Serial.println("Blynk/Internet initialization failed.");
        // internetAvailable = false; // State managed in initializeBlynk now
        if (checkNoInternetMode(ArduinoSerial))
        { // Uses Stream version
            Serial.println("Continuing in No Internet mode as accepted by Arduino.");
        }
        else
        {
            Serial.println("CRITICAL: Arduino did not accept No Internet mode. Halting system.");
            while (1)
            {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                delay(200);
            }
        }
    }
    else
    {
        internetAvailable = true; // Ensure flag is set if connected
    }
    // Setup timers for periodic tasks
    Serial.println("Setting up timers...");
    // Request data from Arduino every BLYNK_SEND_INTERVAL
    timer.setInterval(BLYNK_SEND_INTERVAL, []()
                      {
                          sendSensorDataCMD(ArduinoSerial); // Call the command for the specific port
                      });
    return blynkInitialized;
}

// Helper function to handle common data sending logic
void sendSensorData()
{
    // Check if new data is actually available to be sent
    Serial.println("Processing available sensor data...");

    // Send data to Blynk if network and connection are available
    if (internetAvailable && blynkConnected)
    {
        Serial.println("Sending data to Blynk...");
        // Basic sensor data
        Blynk.virtualWrite(VPIN_TEMP, temperature);
        Blynk.virtualWrite(VPIN_HUMIDITY, humidity);
        Blynk.virtualWrite(VPIN_PRESSURE, pressure); // Assuming VPIN_PRESSURE is defined
        // Need readBattery function here or pass the value
        // Assuming readBattery is available globally or via sensors.h/cpp
        Blynk.virtualWrite(VPIN_BATTERY, ((readBattery(BAT_ADC) / 4200.0) * 100.0)); // Send battery percentage

        // Gas sensor readings
        Blynk.virtualWrite(VPIN_CO_W, v_CO_w);
        Blynk.virtualWrite(VPIN_CO_A, v_CO_a);
        Blynk.virtualWrite(VPIN_SO2_W, v_SO2_w);
        Blynk.virtualWrite(VPIN_SO2_A, v_SO2_a);
        Blynk.virtualWrite(VPIN_NO2_W, v_NO2_w);
        Blynk.virtualWrite(VPIN_NO2_A, v_NO2_a);
        Blynk.virtualWrite(VPIN_OX_W, v_OX_w);
        Blynk.virtualWrite(VPIN_OX_A, v_OX_a);
        Blynk.virtualWrite(VPIN_PID_W, v_pid_w);
        Blynk.virtualWrite(VPIN_CO2_W, v_co2_w);

        // Status information
        bool isArduinoConnected = (millis() - lastCommTime < CMD_TIMEOUT); // Check if Arduino comms recent
        bool isSdOk = SD.begin(SD_CS);                                     // Quick check if SD card is still responding

        Blynk.virtualWrite(VPIN_ARDUINO_STATUS, isArduinoConnected ? 1 : 0);
        Blynk.virtualWrite(VPIN_SD_STATUS, isSdOk ? 1 : 0);
        Serial.println("Data sent to Blynk.");

        // Log data to SD card if logging is enabled
        if (!stopReading)
        {
            Serial.println("Logging data to SD card...");
            logToSD(); // Assuming logToSD is available globally or via sd_card.h/cpp
        }
        else
        {
            Serial.println("SD card logging is disabled.");
        }
    }
    else
    {
        // This part handles the case where sendSensorData might be called periodically
        // even without new data, specifically for checking the communication timeout.
        // Serial.println("No new sensor data available to process."); // Can be noisy, uncomment if needed

        // Handle communication timeout if auto-reset is enabled
        if (autoResetEnabled && (millis() - lastCommTime > CMD_TIMEOUT))
        {
            Serial.print("Arduino communication timeout detected (last comm > ");
            Serial.print(CMD_TIMEOUT / 1000);
            Serial.println("s ago). Resetting system...");
            resetSystem();
        }
    }
}

/**
 * @brief Runs the main Blynk task loop.
 * Should be called repeatedly in the main loop().
 */
void runBlynk() {
    if (blynkConnected) {
        Blynk.run();
    }
    timer.run(); // Run BlynkTimer tasks regardless of connection status?
                 // Maybe timer should only run if connected, depending on tasks.
                 // Current task (sendSensorDataCMD) probably okay to run always.
}

// --- Blynk Event Handlers --- 

BLYNK_WRITE(VPIN_SD_LOGGING)
{
    if (param.asInt() == 1) {
        stopReading = false;
        Serial.println("SD card logging started from Blynk");
        Blynk.logEvent("logging_started"); // Send event notification to Blynk app
    } else {
        stopReading = true;
        Serial.println("SD card logging stopped from Blynk");
        Blynk.logEvent("logging_stopped"); // Send event notification to Blynk app
    }
}

BLYNK_WRITE(VPIN_RESET) {
    if (param.asInt() == 1) {
        Serial.println("Reset requested from Blynk");
        Blynk.logEvent("system_reset"); // Send event notification to Blynk app

        // Send reset command to Arduino first (assuming ArduinoSerial)
        if (sendCommand(ArduinoSerial, CMD_RESET, "", CMD_TIMEOUT)) {
            // Arduino acknowledged the reset command, now reset ESP32
            Serial.println("Arduino acknowledged reset request. Resetting ESP32.");
            resetSystem(); // Assumes resetSystem() is available
        } else {
            Serial.println("Failed to send reset command to Arduino. Aborting ESP32 reset.");
            // Optionally, try resetting ESP32 anyway or log an error
        }
    }
}

BLYNK_WRITE(VPIN_AUTO_RESET) {
    autoResetEnabled = (param.asInt() == 1);
    Serial.print("Auto-reset feature ");
    Serial.println(autoResetEnabled ? "enabled" : "disabled");
    Blynk.logEvent(autoResetEnabled ? "auto_reset_on" : "auto_reset_off"); // Send event notification
}

BLYNK_CONNECTED() {
    Serial.println("Successfully reconnected to Blynk server.");
    blynkConnected = true;
    internetAvailable = true; // Update state on connect
    // Optional: Resync virtual pins if needed upon reconnection
    // Blynk.syncVirtual(VPIN_SD_LOGGING);
    // Blynk.syncVirtual(VPIN_AUTO_RESET);
} 