#include "global.h"
#include "blynk_interface.h"
#include <BlynkSimpleTinyGSM.h>
#include "sensors.h"
#include "sd_card.h"
#include "communication.h" // For sendCommand
#include "system_utils.h"  // For resetSystem
// Define global variables specific to Blynk Interface

char auth[] = BLYNK_AUTH_TOKEN; // Assuming BLYNK_AUTH_TOKEN is in EnvSensor.h
char apn[]  = ""; // Defined here now
char user[] = ""; // Defined here now
char pass[] = ""; // Defined here now
BlynkTimer timer; // Defined here now
bool httpRequestInProgress = false; // File scope global flag



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
    
    // Setup separate timer for VOC data sending to pump ESP32 (every 30 seconds)
    // timer.setInterval(30000, []()
    //                   {
    //                       sendVOCDataToPumpESP32(); // Send VOC data via HTTP API
    //                   });
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
        bool isSdOk = false; // Default to false
        if (!httpRequestInProgress) { // Only check SD card if no HTTP request is active
            isSdOk = SD.begin(SD_CS); // Quick check if SD card is still responding
        } else {
            Serial.println("Skipping SD card status check during HTTP request.");
            // isSdOk remains false, or you could use a previously stored status if available
        }

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

/**
 * @brief Sends sensor data to Blynk dashboard
 */
void sendSensorDataToBlynk() {
    if (!blynkConnected) return;

    // Send sensor data to virtual pins
    Blynk.virtualWrite(VPIN_TEMP, temperature);
    Blynk.virtualWrite(VPIN_HUMIDITY, humidity);
    Blynk.virtualWrite(VPIN_PRESSURE, pressure / 100.0); // Convert Pa to hPa
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

    Serial.println("Sensor data sent to Blynk");
    
    // Note: VOC data sending moved to timer-based system for better stability
    // sendVOCDataToPumpESP32() is now called from timer in initializeBlynk()
}

/**
 * @brief Sends VOC data to pump ESP32 via Blynk API
 */
void sendVOCDataToPumpESP32() {
    if (!blynkConnected) return;
    
    // Emergency disable mechanism - if system is unstable, skip HTTP requests
    static bool httpDisabled = false;
    if (httpDisabled) {
        static unsigned long lastDisableMsg = 0;
        if (millis() - lastDisableMsg > 60000) { // Print message every minute
            Serial.println("HTTP requests temporarily disabled due to system instability");
            lastDisableMsg = millis();
        }
        return;
    }
    
    static unsigned long lastVOCSend = 0;
    static unsigned long lastCrashTime = 0;
    static int consecutiveFailures = 0;
    const unsigned long VOC_SEND_INTERVAL = 30000; // Increased to 30 seconds to reduce load
    
    // If we've had too many failures, temporarily disable HTTP
    if (consecutiveFailures > 5) {
        httpDisabled = true;
        Serial.println("Too many HTTP failures - disabling HTTP requests temporarily");
        return;
    }
    
    // If we've had recent failures, increase the interval
    unsigned long actualInterval = VOC_SEND_INTERVAL;
    if (consecutiveFailures > 0) {
        actualInterval = VOC_SEND_INTERVAL * (1 + consecutiveFailures); // Exponential backoff
        if (actualInterval > 300000) actualInterval = 300000; // Max 5 minutes
    }
    
    if (millis() - lastVOCSend < actualInterval) {
        return; // Not time to send yet
    }
    
    if (httpRequestInProgress) { // Check the global flag
        Serial.println("HTTP request already in progress (checked in sendVOCDataToPumpESP32), skipping this call...");
        return; // Prevent concurrent requests
    }
    
    // Check available memory before making HTTP request
    size_t freeHeap = ESP.getFreeHeap();
    size_t minFreeHeap = ESP.getMinFreeHeap();
    
    if (freeHeap < 25000) { // Increased minimum to 25KB for safety
        Serial.print("Insufficient memory for HTTP request. Free heap: ");
        Serial.print(freeHeap);
        Serial.print(", Min free heap: ");
        Serial.println(minFreeHeap);
        return;
    }
    
    // Additional safety check - if minimum free heap is too low, skip
    if (minFreeHeap < 15000) {
        Serial.print("System memory fragmentation detected. Min free heap: ");
        Serial.println(minFreeHeap);
        return;
    }
    
    Serial.print("Sending VOC data to pump ESP32 via HTTP API: ");
    Serial.print(v_pid_w);
    Serial.print(" (Free heap: ");
    Serial.print(freeHeap);
    Serial.println(" bytes)");
    
    // Pump ESP32 auth tokens for different pump numbers
    String pumpAuthToken = "";
    
    #if PUMP_NUM == 1
    pumpAuthToken = "jiH6wNgCtex-XP0jmEBz5iy2DEDvcrOc";
    #elif PUMP_NUM == 2
    pumpAuthToken = "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI";
    #elif PUMP_NUM == 3
    pumpAuthToken = "tzqMA1jqbtyY2iCwSWi6u34KtkcQKZ0L";
    #elif PUMP_NUM == 4
    pumpAuthToken = "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI";
    #else
    pumpAuthToken = "jiH6wNgCtex-XP0jmEBz5iy2DEDvcrOc"; // Default to pump 1
    #endif
    
    if (pumpAuthToken.length() == 0) {
        Serial.println("Error: No pump auth token configured");
        return;
    }
    
    httpRequestInProgress = true; // Set global flag to prevent concurrent requests
    
    // Check cellular connection status before making HTTP request
    if (!modem.isNetworkConnected()) {
        Serial.println("Error: Cellular network not connected. Cannot send VOC data.");
        httpRequestInProgress = false; // Clear flag on error
        return;
    }
    
    Serial.print("Cellular signal strength: ");
    Serial.println(modem.getSignalQuality());
    
    // Create TinyGSM client for HTTP over cellular
    TinyGsmClient client(modem);
    
    // Use HTTP (port 80) instead of HTTPS to avoid SSL complexity
    String host = "blynk.cloud";
    int port = 80; // HTTP port (simpler than HTTPS)
    String path = "/external/api/update?token=" + pumpAuthToken + "&pin=V16&value=" + String(v_pid_w, 3);
    
    Serial.print("Connecting to: ");
    Serial.print(host);
    Serial.print(":");
    Serial.println(port);
    Serial.print("Path: ");
    Serial.println(path);
    
    // Connect to server with timeout
    if (!client.connect(host.c_str(), port)) {
        Serial.println("Failed to connect to Blynk server via cellular");
        httpRequestInProgress = false; // Clear flag on error
        return;
    }
    
    Serial.println("Connected to Blynk server. Sending HTTP request...");
    
    // Send HTTP GET request manually (simplified)
    String request = "GET " + path + " HTTP/1.1\r\n";
    request += "Host: " + host + "\r\n";
    request += "Connection: close\r\n";
    request += "\r\n";
    
    Serial.print("Request size: ");
    Serial.println(request.length());
    
    client.print(request);
    
    // Wait for response with shorter timeout to avoid memory issues
    unsigned long timeout = millis() + 5000; // Reduced to 5 second timeout
    bool responseReceived = false;
    
    while (millis() < timeout && !responseReceived) {
        // Reset watchdog during HTTP operation
        if (millis() % 1000 == 0) {
            // Reset watchdog every second during HTTP operation
            // Note: This assumes resetWatchdog() is available from system_utils.h
        }
        
        if (client.available()) {
            // Just read the first line to get status code
            String statusLine = client.readStringUntil('\n');
            statusLine.trim();
            
            Serial.print("HTTP Status: ");
            Serial.println(statusLine);
            
            // Simple status check - look for "200" in response
            if (statusLine.indexOf("200") > 0) {
                Serial.println("VOC data sent successfully to pump ESP32 via cellular");
                consecutiveFailures = 0; // Reset failure counter on success
            } else {
                Serial.println("HTTP request may have failed - check status");
                consecutiveFailures++;
            }
            
            responseReceived = true;
            break;
        }
        delay(10); // Small delay to prevent tight loop
    }
    
    if (!responseReceived) {
        Serial.println("HTTP request timeout via cellular");
        consecutiveFailures++; // Increment failure counter on timeout
    }
    
    // Always close connection and cleanup
    client.stop();
    Serial.println("HTTP connection closed");
    
    // Add small delay to ensure cleanup completes
    delay(100);
    
    // Check memory after HTTP operation
    Serial.print("Free heap after HTTP: ");
    Serial.println(ESP.getFreeHeap());
    
    httpRequestInProgress = false; // Clear global flag when done
    lastVOCSend = millis();
    
    Serial.println("VOC HTTP request completed successfully");
}