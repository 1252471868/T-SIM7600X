/*
  ESP32 Environmental Monitoring System with Blynk Cloud Integration
  
  This code combines the sensor reading logic from the original Arduino code
  with remote connectivity through Blynk on ESP32.
  
  Features:
  - Gas sensors reading (CO, CO2, NO2, O3, SO2)
  - Temperature, humidity, pressure reading
  - SD card logging
  - Real-time clock synchronization
  - Remote control through Blynk dashboard
*/

#include "EnvSensor.h"
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <esp_task_wdt.h>  // Include for watchdog timer

// #define DUMP_AT_COMMANDS // Uncomment for detailed AT command debugging

// Blynk & GPRS credentials
char auth[] = BLYNK_AUTH_TOKEN;
char apn[] = "";  // Your APN
char user[] = ""; // APN username if any
char pass[] = ""; // APN password if any

// Global objects
Adafruit_BME680 bme(&Wire); // BME680 sensor object
BlynkTimer timer;           // Blynk timer object
File SDstorage;              // File object for SD card logging
String filename = "/DAT";     // Base filename for SD card logs (updated in setupSD)
volatile bool stopReading = false; // Flag to control SD card logging (controlled via Blynk V3)

// Communication control variables
volatile bool internetAvailable = true; // Flag indicating network availability
volatile bool dataAvailable = false;    // Flag indicating new sensor data is ready from Arduino
volatile bool autoResetEnabled = AUTO_RESET_ENABLED; // Flag for auto-reset feature (controlled via Blynk V15)
unsigned long lastCommTime = 0;         // Timestamp of the last successful communication with Arduino
bool blynkConnected = false;        // Flag indicating connection status to Blynk server

// Placeholder definitions to satisfy linter - Actual values should be in EnvSensor.h
#ifndef COMM_TIMEOUT
#define COMM_TIMEOUT 60000 // Default communication timeout (e.g., 60 seconds)
#endif
#ifndef WDT_TIMEOUT
#define WDT_TIMEOUT 60 // Default watchdog timeout 60s
#endif
#ifndef RECONNECT_INTERVAL
#define RECONNECT_INTERVAL 1200000 // Default reconnect interval 20 minutes (20 * 60 * 1000)
#endif

// HardwareSerial for communication with Arduino Mega
HardwareSerial ArduinoSerial(2); // Using UART2 (GPIO 16=RX, 17=TX)

// For debugging AT commands
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT); // TinyGSM modem object
#endif

// Sensor data variables
float temperature, humidity, pressure;
double v_CO_w, v_CO_a, v_SO2_w, v_SO2_a;
double v_NO2_w, v_NO2_a, v_OX_w, v_OX_a;
double v_pid_w, v_co2_w;

/**
 * @brief Blynk Virtual Pin V3 handler.
 * Controls the starting and stopping of SD card logging based on the value received.
 * 
 * @param param BlynkParam object containing the value written to V3.
 */
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

/**
 * @brief Blynk Virtual Pin V14 handler.
 * Initiates a system reset (both Arduino and ESP32) when triggered.
 * 
 * @param param BlynkParam object (value is checked, but not used beyond triggering).
 */
BLYNK_WRITE(VPIN_RESET) {
  if (param.asInt() == 1) {
    Serial.println("Reset requested from Blynk");
    Blynk.logEvent("system_reset"); // Send event notification to Blynk app
    
    // Send reset command to Arduino first
    if (sendCommand(CMD_RESET, "", CMD_TIMEOUT)) {
      // Arduino acknowledged the reset command, now reset ESP32
      Serial.println("Arduino acknowledged reset request. Resetting ESP32.");
      resetSystem();
    } else {
      Serial.println("Failed to send reset command to Arduino. Aborting ESP32 reset.");
      // Optionally, try resetting ESP32 anyway or log an error
    }
  }
}

/**
 * @brief Blynk Virtual Pin V15 handler.
 * Enables or disables the automatic system reset feature based on communication timeout.
 * 
 * @param param BlynkParam object containing the toggle state (1=enabled, 0=disabled).
 */
BLYNK_WRITE(VPIN_AUTO_RESET) {
  autoResetEnabled = (param.asInt() == 1);
  Serial.print("Auto-reset feature ");
  Serial.println(autoResetEnabled ? "enabled" : "disabled");
  Blynk.logEvent(autoResetEnabled ? "auto_reset_on" : "auto_reset_off"); // Send event notification
}

/**
 * @brief Callback function executed when the ESP32 successfully connects to the Blynk server.
 * Synchronizes the state of virtual pins V3 (SD logging) and V15 (auto-reset) with the server.
 */
BLYNK_CONNECTED() {
  Serial.println("Successfully connected to Blynk server.");
  blynkConnected = true;
  // Blynk.syncVirtual(V3);  // Sync the SD logging state from the server
  // Blynk.syncVirtual(V15); // Sync the auto-reset state from the server
}

/**
 * @brief Reads the voltage from the specified analog pin and calculates the battery voltage.
 * Assumes a voltage divider circuit is used.
 * 
 * @param pin The analog pin connected to the battery voltage sense point.
 * @return float The calculated battery voltage.
 */
float readBattery(uint8_t pin) {
  int vref = 1100; // Internal reference voltage (may need calibration)
  uint16_t adcValue = analogRead(pin);
  // Calculation based on ESP32 ADC range (0-4095), 3.3V ref, 2x divider, and vref scaling
  // Adjust formula based on your specific voltage divider and ADC setup
  float battery_voltage = ((float)adcValue / 4095.0) * 2.0 * 3.3 * (vref / 1000.0); 
  return battery_voltage;
}

// JSON document for parsing commands
JsonDocument doc; 

/**
 * @brief Processes incoming commands from the Arduino Mega via UART2.
 * 
 * Reads serial data, looks for a complete JSON object representing a command,
 * parses the JSON, and acts based on the command received (e.g., RESET, DATA).
 */
void processIncomingCommands() {
  if (ArduinoSerial.available()) {
    String jsonString = "";
    bool foundStart = false;
    bool foundEnd = false;
    
    // Simple state machine to find '{' ... '}' structure
    unsigned long readStartTime = millis();
    while (ArduinoSerial.available()) { // Timeout for reading JSON
      char c = ArduinoSerial.read();
      
      if (!foundStart) {
        if (c == '{') {
          foundStart = true;
          jsonString = c;
        }
        // Discard characters before '{'
      } else {
        jsonString += c;
        if (c == '}') {
          foundEnd = true;
          break; // Found complete JSON object
        }
      }
    }

    // Process only if a complete JSON object was potentially found
    if (foundStart && foundEnd) {
      Serial.println("Received potential JSON from Arduino: " + jsonString);
      
      // Parse JSON command
      JsonDocument cmdDoc;
      DeserializationError error = deserializeJson(cmdDoc, jsonString);
      
      if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return; // Exit if parsing fails
      }
      
      // Extract command string
      String command = cmdDoc["cmd"].as<String>();
      
      // Update last communication time upon receiving any valid command
      lastCommTime = millis();
      Serial.print("Received command: "); Serial.println(command);
      
      // Handle different commands
      if (command == CMD_RESET) {
        // Arduino is notifying about its reset
        Serial.println("Arduino has reset. Sending ACK.");
        sendCommandWithoutResponse(CMD_ACK, ""); // Acknowledge Arduino's reset notification
      }
      else if (command == CMD_DATA) {
        // Arduino sent sensor data
        Serial.println("Processing sensor data from Arduino...");
        JsonArray dataArray = cmdDoc["data"].as<JsonArray>();
        
        // Check if the data array has the expected number of elements
        if (dataArray.size() >= 15) {
          // Extract values from JSON array based on expected order
          temperature = dataArray[1]; 
          humidity = dataArray[2];    
          pressure = dataArray[3];    
          v_CO_w = dataArray[5];     
          v_CO_a = dataArray[6];     
          v_SO2_w = dataArray[7];    
          v_SO2_a = dataArray[8];    
          v_NO2_w = dataArray[9];    
          v_NO2_a = dataArray[10];   
          v_OX_w = dataArray[11];    
          v_OX_a = dataArray[12];   
          v_pid_w = dataArray[13];   
          v_co2_w = dataArray[14];   
          
          dataAvailable = true; // Set flag indicating new data is ready
          Serial.println("Sensor data received successfully. Sending ACK.");
          sendCommandWithoutResponse(CMD_ACK, ""); // Acknowledge data reception
          // Process and send data immediately after acknowledging
          sendSensorData(); 
        } else {
          Serial.print("Incomplete sensor data received. Expected >=15 items, got: ");
          Serial.println(dataArray.size());
          sendCommandWithoutResponse(CMD_ACK, ""); // Acknowledge anyway, maybe Arduino can retry
        }
      } else {
         Serial.print("Unknown command received: "); Serial.println(command);
         // Optionally send an error response
      }
    } else if (foundStart && !foundEnd) {
       Serial.println("Incomplete JSON received (no closing '}'). Discarding.");
    } // Ignore if no '{' was found
  }
}

/**
 * @brief Sends a command to the Arduino Mega without waiting for a response.
 * Formats the command and data into a JSON string and sends it via UART2.
 * 
 * @param cmd The command string (e.g., CMD_ACK).
 * @param data The data string associated with the command (can be empty).
 */
void sendCommandWithoutResponse(const char* cmd, const String& data) {
  JsonDocument cmdDoc;
  cmdDoc["cmd"] = cmd;
  cmdDoc["data"] = data;
  String jsonString;
  serializeJson(cmdDoc, jsonString);
  
  ArduinoSerial.println(jsonString);
  Serial.print("Sent command (no response wait): ");
  Serial.println(jsonString);
}

/**
 * @brief Sends a command to the Arduino Mega and waits for a JSON response.
 * Formats the command and data into JSON, sends it via UART2, and waits
 * for a JSON response containing a "cmd" field within the specified timeout.
 * 
 * @param cmd The command string to send.
 * @param data The data string associated with the command.
 * @param timeout The maximum time in milliseconds to wait for a response.
 * @return true If a valid JSON response with a "cmd" field was received within the timeout.
 * @return false If the timeout occurred or JSON parsing failed.
 */
bool sendCommand(const char* cmd, const String& data, unsigned long timeout) {
  JsonDocument cmdDoc;
  cmdDoc["cmd"] = cmd;
  cmdDoc["data"] = data;
  
  String jsonStringToSend;
  serializeJson(cmdDoc, jsonStringToSend);
  
  Serial.print("Sending command (expect response): ");
  Serial.println(jsonStringToSend);
  
  // Send the command
  ArduinoSerial.println(jsonStringToSend);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < timeout) {
    if (ArduinoSerial.available()) {
      // Attempt to read a JSON response
      String jsonResponseString = "";
      bool foundRespStart = false;
      bool foundRespEnd = false;
      
      while (ArduinoSerial.available()) { // Inner timeout for reading
        char c = ArduinoSerial.read();
        if (!foundRespStart) {
            if (c == '{') {
                foundRespStart = true;
                jsonResponseString = c;
            }
        } else {
            jsonResponseString += c;
            if (c == '}') {
                foundRespEnd = true;
                break;
            }
        }
      }
      
      // Process if a potential JSON object was received
      if (foundRespStart && foundRespEnd) {
        Serial.println("Received potential JSON response: " + jsonResponseString);
        JsonDocument respDoc;
        DeserializationError error = deserializeJson(respDoc, jsonResponseString);
        
        if (!error && respDoc.containsKey("cmd")) { // Check for parsing error and presence of "cmd" field
          response = respDoc["cmd"].as<String>();
          Serial.print("Received valid response command: "); Serial.println(response);
          lastCommTime = millis(); // Update last successful communication time
          return true; // Success
        } else {
            Serial.print("JSON response parsing failed or 'cmd' key missing. Error: ");
            Serial.println(error.c_str());
            // Continue waiting for a potentially valid response
        }
      }
    }
    delay(10); // Small delay to prevent busy-waiting
  }
  
  // If loop finishes without returning true, it's a timeout
  Serial.println("Command response timeout!");
  return false;
}

/**
 * @brief Sends a command to the Arduino Mega with a specified number of retries.
 * Uses the sendCommand function and retries if it returns false.
 * 
 * @param cmd The command string to send.
 * @param data The data string associated with the command.
 * @param maxRetries The maximum number of times to retry sending the command.
 * @return true If the command was successfully sent and acknowledged within the retry limit.
 * @return false If the command failed even after all retries.
 */
bool sendCommandWithRetry(const char* cmd, const String& data, int maxRetries) {
  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    if (sendCommand(cmd, data, CMD_TIMEOUT)) {
      return true; // Success
    }
    // Log retry attempt if failed
    Serial.print("Command failed. Retry attempt ");
    Serial.print(attempt);
    Serial.print(" of ");
    Serial.println(maxRetries);
    delay(500); // Delay before retrying
  }
  
  // Log failure after all retries
  Serial.print("Failed to send command '");
  Serial.print(cmd); 
  Serial.print("' after ");
  Serial.print(maxRetries);
  Serial.println(" attempts.");
  return false;
}

/**
 * @brief Performs a software reset of the ESP32.
 * Logs a message and then calls ESP.restart().
 */
void resetSystem() {
  Serial.println("Performing ESP32 system reset...");
  delay(500); // Short delay to allow logging
  esp_task_wdt_deinit(); // Deinitialize watchdog before reset
  ESP.restart();
}

/**
 * @brief Checks if the Arduino Mega accepts operation in "No Internet" mode.
 * Sends the CMD_NONET command and expects an acknowledgment (implicitly handled by sendCommand).
 * 
 * @return true If the Arduino acknowledges the CMD_NONET command.
 * @return false If the command fails or times out.
 */
bool checkNoInternetMode() {
  Serial.println("Querying Arduino about No Internet mode acceptance...");
  // sendCommand handles the response check (expects ACK or similar positive response)
  if (sendCommand(CMD_NONET, "", CMD_TIMEOUT)) { 
    Serial.println("Arduino accepted No Internet mode.");
    return true;
  } else {
    Serial.println("Arduino did not accept or respond to No Internet mode query.");
    return false;
  }
}

/**
 * @brief Verifies communication with the Arduino Mega by sending an INFO command.
 * Attempts to establish communication within a specified timeout, blinking an LED
 * during the process. Retries sending the command multiple times.
 * 
 * @return true If communication is successfully established (Arduino responds).
 * @return false If communication cannot be established within the timeout.
 */
bool verifyArduinoCommunication() {
  Serial.println("Attempting to establish communication with Arduino Mega...");
  
  unsigned long startTime = millis();
  const unsigned long verificationTimeout = 30000; // 30 seconds timeout for verification
  const int maxAttempts = 5; // Number of attempts within the timeout period
  
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    Serial.print("Verification attempt "); Serial.println(attempt);
    // Blink LED rapidly during verification attempt
    for (int i=0; i<5; ++i) { 
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); 
    }
    digitalWrite(LED_PIN, LOW); // Leave LED off between main attempts

    // Try sending the INFO command
    if (sendCommand(CMD_INFO, "", CMD_TIMEOUT)) {
      Serial.println("Communication with Arduino Mega established successfully!");
      lastCommTime = millis(); // Update communication time
      digitalWrite(LED_PIN, HIGH); // Turn LED solid on to indicate success
      return true;
    }
    
    // Check if overall timeout exceeded
    if (millis() - startTime > verificationTimeout) {
        break; // Exit loop if main timeout reached
    }
    
    Serial.println("Verification attempt failed. Waiting before next attempt...");
    delay(2000); // Wait longer between main verification attempts
  }
  
  // If loop completes without success
  Serial.println("Failed to establish communication with Arduino Mega after multiple attempts!");
  digitalWrite(LED_PIN, LOW); // Ensure LED is off on failure
  return false;
}

/**
 * @brief Sends the CMD_DATA command to the Arduino without waiting for a response.
 * This requests the Arduino to send its latest sensor readings.
 */
void sendSensorDataCMD() {
  Serial.println("Requesting sensor data from Arduino...");
  sendCommandWithoutResponse(CMD_DATA, "");
}

/**
 * @brief Processes available sensor data: sends it to Blynk and logs it to SD card.
 * This function is called after new data is received from the Arduino
 * (indicated by the dataAvailable flag) or potentially by a timer.
 * Also handles auto-reset logic if communication with Arduino is lost.
 */
void sendSensorData() {
  // Check if new data is actually available to be sent
  if (dataAvailable) {
    Serial.println("Processing available sensor data...");
    
    // Send data to Blynk if network and connection are available
    if (internetAvailable && blynkConnected) {
      Serial.println("Sending data to Blynk...");
      // Basic sensor data
      Blynk.virtualWrite(VPIN_TEMP, temperature);
      Blynk.virtualWrite(VPIN_HUMIDITY, humidity);
      Blynk.virtualWrite(VPIN_BATTERY, ((readBattery(BAT_ADC) / 4200.0) * 100.0)); // Send battery percentage
      
      // Gas sensor readings
      Blynk.virtualWrite(VPIN_CO_W, v_CO_w);   Blynk.virtualWrite(VPIN_CO_A, v_CO_a);
      Blynk.virtualWrite(VPIN_SO2_W, v_SO2_w);  Blynk.virtualWrite(VPIN_SO2_A, v_SO2_a);
      Blynk.virtualWrite(VPIN_NO2_W, v_NO2_w);  Blynk.virtualWrite(VPIN_NO2_A, v_NO2_a);
      Blynk.virtualWrite(VPIN_OX_W, v_OX_w);  Blynk.virtualWrite(VPIN_OX_A, v_OX_a);
      Blynk.virtualWrite(VPIN_PID_W, v_pid_w);
      Blynk.virtualWrite(VPIN_CO2_W, v_co2_w);
      
      // Status information
      bool isArduinoConnected = (millis() - lastCommTime < COMM_TIMEOUT); // Check if Arduino comms recent
      bool isSdOk = SD.begin(SD_CS); // Quick check if SD card is still responding
      
      Blynk.virtualWrite(VPIN_ARDUINO_STATUS, isArduinoConnected ? 1 : 0);
      Blynk.virtualWrite(VPIN_SD_STATUS, isSdOk ? 1 : 0);
      Serial.println("Data sent to Blynk.");
    } else {
      Serial.println("Skipping Blynk data send (Internet or Blynk connection unavailable).");
    }
    
    // Log data to SD card if logging is enabled
    if (!stopReading) {
      Serial.println("Logging data to SD card...");
      logToSD();
    } else {
      Serial.println("SD card logging is disabled.");
    }
    
    // Reset the flag after processing the data
    dataAvailable = false;

  } else {
    // This part handles the case where sendSensorData might be called periodically 
    // even without new data, specifically for checking the communication timeout.
    // Serial.println("No new sensor data available to process."); // Can be noisy, uncomment if needed
    
    // Handle communication timeout if auto-reset is enabled
    if (autoResetEnabled && (millis() - lastCommTime > COMM_TIMEOUT)) { 
      Serial.print("Arduino communication timeout detected (last comm > ");
      Serial.print(COMM_TIMEOUT / 1000); Serial.println("s ago). Resetting system...");
      resetSystem();
    }
  }
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
          // Set the ESP32 system time
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

/**
 * @brief Creates a new data filename for the SD card.
 * 
 * Generates a filename based on the current date and time in the format
 * "/boxX_YYYYMMDD HHMMSS.txt" if the system time is valid (year > 2020).
 * If the time is not valid, it falls back to a numbered filename format
 * "/boxX_DATn.txt", incrementing 'n' until an unused filename is found.
 * 
 * @return String The generated filename, including the leading slash.
 */
String createDataFilename() {
  char dateTimeFilename[30];
  bool timeValid = (year() > 2020); // Basic check if system time seems valid
  String newFilename;
  
  if (timeValid) {
    // Time is valid, use timestamp format with box number prefix
    sprintf(dateTimeFilename, "/box%d_%04d%02d%02d_%02d%02d%02d.txt", 
            BOX_NUM, year(), month(), day(), hour(), minute(), second());
    newFilename = String(dateTimeFilename);
    Serial.println("Using timestamp filename: " + newFilename);
  } else {
    // Time not set, use fallback numbered format with box number prefix
    Serial.println("Time not set, using fallback filename format.");
    bool fileFound = false;
    int count = 0;
    String baseFilename = "/box" + String(BOX_NUM) + "_DAT";
    while (!fileFound) {
      String testFilename = baseFilename + String(count) + ".txt";
      if (SD.exists(testFilename)) {
        count++;
      } else {
        newFilename = testFilename;
        fileFound = true;
      }
    }
    Serial.println("Using fallback filename: " + newFilename);
  }
  
  return newFilename;
}

/**
 * @brief Initializes the SD card and creates the initial data log file.
 * 
 * Sets up the SPI communication for the SD card, initializes the SD library,
 * creates the first data filename using createDataFilename(), and opens
 * the file, writing the header row.
 */
void setupSD() {
  Serial.println("Initializing SD card...");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS); // Initialize SPI pins
  
  // Attempt to initialize SD card
  if (!SD.begin(SD_CS)) { 
    Serial.println("SD card initialization failed! Check connection and formatting.");
    // Consider halting setup or using an error LED pattern
    return; // Cannot proceed without SD card
  }
  
  Serial.println("SD card initialized successfully.");
  
  // Create the initial filename for logging
  filename = createDataFilename();
  
  // Open the initial file for writing (creates if not exists, truncates if exists)
  SDstorage = SD.open(filename.c_str(), FILE_WRITE);
  if (SDstorage) {
    Serial.println("Opened initial data file: " + filename);
    // Write the header row to the new file
    SDstorage.println("Time,Temperature(C),Humidity(%),Pressure(hPa),CO_W,CO_A,SO2_W,SO2_A,NO2_W,NO2_A,OX_W,OX_A,PID_W,CO2_W"); // Using CSV format
    SDstorage.flush(); // Ensure header is written immediately
    Serial.println("Wrote header to data file.");
  } else {
    Serial.println("Failed to open initial data file for writing! Check SD card.");
    // Consider halting setup or using an error LED pattern
  }
}

/**
 * @brief Attempts to reconnect to the SD card if initialization fails.
 * Loops indefinitely until SD.begin() succeeds.
 */
void reconnectSD() {
  Serial.println("Attempting to reconnect SD card...");
  while (!SD.begin(SD_CS)) {
    Serial.print("."); // Print dots while waiting
    delay(500);
  }
  Serial.println("\nSD card reconnected successfully.");
}

/**
 * @brief Initializes the cellular modem (SIM7600).
 * Sets up UART communication, toggles power and flight mode pins,
 * attempts to restart the modem up to 5 times, and logs the modem name.
 */
void setupModem() {
  Serial.println("Initializing modem...");
  // Begin UART communication with the modem
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
 * @brief Logs the current sensor data to the SD card.
 * 
 * Handles SD card reconnection, file size checking (creates new file if > MAX_FILE_SIZE),
 * and writing the data row in CSV format. Includes watchdog timer resets.
 */
void logToSD() {
  // Reset watchdog timer before potentially long SD operations
  esp_task_wdt_reset();
  
  // 1. Ensure SDstorage object is valid and file is open
  // Check if the currently intended file exists. If not, SD might have been removed/reset.
  if (!SD.exists(filename.c_str())) {
    Serial.println("Current log file not found. Attempting SD reconnect/reopen...");
    reconnectSD(); // Try to re-initialize SD communication
    // Attempt to reopen the intended file in APPEND mode
    SDstorage = SD.open(filename.c_str(), FILE_APPEND);
    if (!SDstorage) {
        Serial.println("Failed to reopen file after reconnect! Trying to create new file...");
        // If reopening fails, try creating a completely new file
        filename = createDataFilename(); 
        SDstorage = SD.open(filename.c_str(), FILE_WRITE);
        if(SDstorage) {
            SDstorage.println("Time,Temperature(C),Humidity(%),Pressure(hPa),CO_W,CO_A,SO2_W,SO2_A,NO2_W,NO2_A,OX_W,OX_A,PID_W,CO2_W");
            SDstorage.flush();
            Serial.println("Created new file after reopen failure: " + filename);
        } else {
            Serial.println("CRITICAL: Failed to open any file for logging!");
            return; // Cannot log
        }
    } else {
         Serial.println("Successfully reopened file: " + filename);
    }
  } else if (!SDstorage) {
      // If file exists but SDstorage is somehow invalid, try reopening it
      Serial.println("SDstorage object invalid, attempting to reopen file: " + filename);
      SDstorage = SD.open(filename.c_str(), FILE_APPEND);
      if (!SDstorage) {
          Serial.println("Failed to reopen existing file!");
          return; // Cannot log
      }
  }
  
  // 2. Proceed only if the file handle (SDstorage) is valid
  if (SDstorage) {
    unsigned long fileSize = SDstorage.size();
    // Serial.printf("Current file size: %lu bytes\n", fileSize); // Verbose logging
    
    // 3. Check if file size exceeds the limit
    if (fileSize > MAX_FILE_SIZE) {
      Serial.printf("File size limit (%lu bytes) reached. Creating new file.\n", MAX_FILE_SIZE);
      SDstorage.close(); // Close the current large file
      
      // Create a new filename
      filename = createDataFilename();
      
      // Open the new file (Write mode implicitly creates/truncates)
      SDstorage = SD.open(filename.c_str(), FILE_WRITE);
      if (SDstorage) {
        Serial.println("Created and opened new data file: " + filename);
        // Write the header to the new file
        SDstorage.println("Time,Temperature(C),Humidity(%),Pressure(hPa),CO_W,CO_A,SO2_W,SO2_A,NO2_W,NO2_A,OX_W,OX_A,PID_W,CO2_W");
        SDstorage.flush(); // Ensure header is written
      } else {
        Serial.println("CRITICAL: Failed to create new data file after size limit reached!");
        return; // Cannot log if new file creation fails
      }
    }
    
    // 4. Format the timestamp for the log entry
    char timeStr[25];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", // ISO-like format is better for logs
            year(), month(), day(), hour(), minute(), second());
    
    // 5. Record start time for write operation timing
    unsigned long writeStartTime = millis();
    
    // 6. Write the data row in CSV format
    SDstorage.print(timeStr);           SDstorage.print(",");
    SDstorage.print(temperature, 4);    SDstorage.print(",");
    SDstorage.print(humidity, 4);       SDstorage.print(",");
    SDstorage.print(pressure / 100.0, 4); SDstorage.print(","); // Convert Pa to hPa for header consistency
    SDstorage.print(v_CO_w, 4);         SDstorage.print(",");
    SDstorage.print(v_CO_a, 4);         SDstorage.print(",");
    SDstorage.print(v_SO2_w, 4);        SDstorage.print(",");
    SDstorage.print(v_SO2_a, 4);        SDstorage.print(",");
    SDstorage.print(v_NO2_w, 4);        SDstorage.print(",");
    SDstorage.print(v_NO2_a, 4);        SDstorage.print(",");
    SDstorage.print(v_OX_w, 4);         SDstorage.print(",");
    SDstorage.print(v_OX_a, 4);         SDstorage.print(",");
    SDstorage.print(v_pid_w, 4);        SDstorage.print(",");
    SDstorage.println(v_co2_w, 4);
    
    // 7. Flush data to SD card to ensure it's written
    SDstorage.flush();
    
    // 8. Check if the write operation took an unusually long time
    unsigned long writeDuration = millis() - writeStartTime;
    if (writeDuration > 5000) { // Threshold for warning (5 seconds)
      Serial.printf("WARNING: SD write/flush operation took %lu ms!\n", writeDuration);
    }
    
    // 9. Reset watchdog timer again after successful SD operations
    esp_task_wdt_reset();
  } else {
    // This should ideally not happen if file opening logic works
    Serial.println("SD file handle (SDstorage) is invalid, cannot log data!");
  }
}

/**
 * @brief Attempts to reconnect to GPRS and Blynk.
 * This function is called periodically by a timer.
 */
void reconnectInternetBlynk() {
  esp_task_wdt_reset(); // Reset watchdog at the beginning
  Serial.println("Checking Internet/Blynk connection status...");

  if (!modem.isNetworkConnected()) {
    Serial.println("GPRS network is not connected. Attempting to reconnect...");
    internetAvailable = false; // Assume not available until successfully reconnected
    blynkConnected = false;  // Blynk cannot be connected if GPRS is down

    if (Blynk.connectNetwork(apn, user, pass)) {
      Serial.println("GPRS reconnected successfully.");
      internetAvailable = true; // GPRS is back
      // Now try to connect to Blynk
      Serial.println("Attempting to connect to Blynk server...");
      if (Blynk.connect(BLYNK_CONNECT_TIMEOUT)) {
        Serial.println("Successfully reconnected to Blynk!");
        blynkConnected = true;
        sendCommand(CMD_CONNECTED, "", CMD_TIMEOUT); // Notify Arduino
      } else {
        Serial.println("Failed to reconnect to Blynk server after GPRS re-established.");
        blynkConnected = false; // Mark Blynk as disconnected
      }
    } else {
      Serial.println("Failed to reconnect to GPRS network.");
      internetAvailable = false;
      blynkConnected = false;
    }
  } else if (!Blynk.connected()) { // GPRS is connected, but Blynk is not
    Serial.println("GPRS connected, but Blynk is not. Attempting to reconnect Blynk...");
    internetAvailable = true; // GPRS is okay
    blynkConnected = false; // Mark Blynk as disconnected before attempting reconnect

    if (Blynk.connect(BLYNK_CONNECT_TIMEOUT)) {
      Serial.println("Successfully reconnected to Blynk!");
      blynkConnected = true;
      sendCommand(CMD_CONNECTED, "", CMD_TIMEOUT); // Notify Arduino
    } else {
      Serial.println("Failed to reconnect to Blynk server.");
      blynkConnected = false;
    }
  } else {
    Serial.println("Internet and Blynk connection appear to be active.");
    // Ensure flags are consistent if somehow they got out of sync
    if (!internetAvailable) internetAvailable = true;
    if (!blynkConnected) blynkConnected = true;
  }

  // Check if reconnection attempts failed overall
  if (!blynkConnected) {
    Serial.println("Failed to establish Blynk connection after reconnection attempts. Resetting ESP32.");
    resetSystem(); // Reset the ESP32 if still not connected
  }

  esp_task_wdt_reset(); // Reset watchdog at the end
}

/**
 * @brief Main setup function, runs once on boot.
 * Initializes serial communication, watchdog, LED, Arduino communication,
 * modem, network time, SD card, Blynk connection, and timers.
 */
void setup() {
  // Initialize serial console for debugging
  Serial.begin(UART_BAUD);
  delay(1000); // Wait for serial monitor connection
  Serial.println("\n--- ESP32 Environmental Sensor System Starting ---");
  
  // Configure and enable watchdog timer
  Serial.println("Initializing watchdog timer...");
  esp_task_wdt_init(WDT_TIMEOUT, true); // Use timeout from header, panic on timeout
  esp_task_wdt_add(NULL);     // Add current task (loop) to watchdog
  esp_task_wdt_reset();       // Reset watchdog initially

  // Initialize indicator LED (set high initially)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize UART for Arduino communication
  Serial.println("Initializing UART for Arduino communication...");
  ArduinoSerial.begin(115200, SERIAL_8N1, ARDUINO_UART_RX, ARDUINO_UART_TX);
  delay(500); // Allow UART to settle
  
  // Try to verify communication with Arduino
  bool isArduinoCommOk = verifyArduinoCommunication();
  if (!isArduinoCommOk) {
    Serial.println("Warning: Initial Arduino communication failed. System will continue, but data may be unavailable.");
    // Keep LED solid HIGH from init, or use a specific pattern (e.g., slow blink)
    // digitalWrite(LED_PIN, LOW); // Example: Turn off if failed
  }
  
  // Initialize modem
  setupModem();
  esp_task_wdt_reset(); // Reset watchdog after modem setup
  
  // Get and set time from network
  bool timeSet = setupTimeWithRetry();
  if (!timeSet) {
      Serial.println("Warning: Failed to set system time. SD filenames will use fallback format.");
  }
  esp_task_wdt_reset(); // Reset watchdog after time setup

  // Setup SD card
  setupSD();
  esp_task_wdt_reset(); // Reset watchdog after SD setup

  // Initialize Blynk connection
  Serial.println("Initializing Blynk connection...");
  Blynk.config(modem, auth, BLYNK_DEFAULT_DOMAIN, BLYNK_DEFAULT_PORT);
  Serial.println("Connecting to GPRS network...");
  if(Blynk.connectNetwork(apn, user, pass)) {
      Serial.println("GPRS network connected. Connecting to Blynk...");
      internetAvailable = true; // GPRS connected
      if (Blynk.connect(BLYNK_CONNECT_TIMEOUT)) { // Timeout for Blynk connection
          blynkConnected = true; // Set global flag only on successful connect
          Serial.println("Successfully connected to Blynk!");
          sendCommand(CMD_CONNECTED, "", CMD_TIMEOUT); // Notify Arduino
      } else {
          Serial.println("Failed to connect to Blynk server.");
          blynkConnected = false; // Ensure it's false
      }
  } else {
      Serial.println("Failed to connect to GPRS network.");
      internetAvailable = false; // GPRS connection failed
      blynkConnected = false;    // Blynk cannot be connected
  }
  esp_task_wdt_reset(); // Reset watchdog after Blynk attempt

  // Fallback logic if Blynk connection failed initially
  if (!blynkConnected) { // Check the global blynkConnected flag
    Serial.println("Initial Blynk connection failed.");
    // Check if Arduino allows operation without internet
    if (checkNoInternetMode()) {
      internetAvailable = false; // Explicitly set as we are in no internet mode
      Serial.println("Continuing in No Internet mode as accepted by Arduino.");
    } else {
      // Arduino requires internet or didn't respond - critical failure
      Serial.println("CRITICAL: Arduino did not accept No Internet mode. System will attempt periodic reconnections.");
      // System will not halt, but rely on reconnectInternetBlynk
      // Consider a more prominent visual indicator for this state if needed
    }
  }
  
  // Setup timers for periodic tasks
  Serial.println("Setting up timers...");
  // Request data from Arduino every BLYNK_SEND_INTERVAL (e.g., 10 seconds)
  timer.setInterval(BLYNK_SEND_INTERVAL, sendSensorDataCMD); 
  // Add timer for periodic internet/Blynk reconnection check
  timer.setInterval(RECONNECT_INTERVAL, reconnectInternetBlynk); // Check every 20 minutes
  // Note: processIncomingCommands is called directly in loop() for responsiveness
  // timer.setInterval(COMM_CHECK_MS, processIncomingCommands); // Alternative: run command processing on timer

  Serial.println("--- Setup Complete --- ");
  digitalWrite(LED_PIN, LOW); // Turn LED off to indicate setup complete and running
}

/**
 * @brief Main loop function, runs repeatedly.
 * Handles Blynk communication, timer events, processes incoming Arduino commands,
 * and resets the watchdog timer.
 */
void loop() {
  // Reset watchdog timer at the start of each loop iteration
  esp_task_wdt_reset(); 

  // Run Blynk tasks if internet is available AND blynk is actually connected
  if (internetAvailable && blynkConnected) {
    Blynk.run();
  }
  
  // Run tasks scheduled by BlynkTimer (e.g., sendSensorDataCMD, reconnectInternetBlynk)
  timer.run();
  
  // Continuously process incoming commands from Arduino
  processIncomingCommands();
  
  // Small delay to prevent loop from running too fast and potentially starving other tasks
  // Adjust delay based on system responsiveness and needs
  // delay(10); 
} 