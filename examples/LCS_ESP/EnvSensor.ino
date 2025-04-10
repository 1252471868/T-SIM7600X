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

// #define DUMP_AT_COMMANDS

// Blynk & GPRS credentials
char auth[] = BLYNK_AUTH_TOKEN;
char apn[] = "";  // Your APN
char user[] = ""; // APN username if any
char pass[] = ""; // APN password if any

// Global objects
Adafruit_BME680 bme(&Wire);
BlynkTimer timer;
File SDstorage;
String filename = "/DAT";  // Add leading slash
volatile bool stopReading = false;

// Communication control variables
volatile bool internetAvailable = true;
volatile bool autoResetEnabled = AUTO_RESET_ENABLED;
unsigned long lastCommTime = 0;
bool blynkConnected = false;

// For debugging
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// Sensor data variables
float temperature, humidity, pressure;
double v_CO_w, v_CO_a, v_SO2_w, v_SO2_a;
double v_NO2_w, v_NO2_a, v_OX_w, v_OX_a;
double v_pid_w, v_co2_w;

// Remote control of SD card logging via Blynk
BLYNK_WRITE(V3) {
  if (param.asInt() == 1) {
    stopReading = false;
    Serial.println("SD card logging started from Blynk");
    Blynk.logEvent("logging_started"); // Send event notification
  } else {
    stopReading = true;
    Serial.println("SD card logging stopped from Blynk");
    Blynk.logEvent("logging_stopped"); // Send event notification
  }
}

// Remote reset control via Blynk
BLYNK_WRITE(V14) {
  if (param.asInt() == 1) {
    Serial.println("Reset requested from Blynk");
    Blynk.logEvent("system_reset"); // Send event notification
    
    // Send reset command to Arduino
    if (sendCommand(CMD_RESET, "", CMD_TIMEOUT)) {
      // Arduino acknowledged, now reset ESP32
      resetSystem();
    }
  }
}

// Remote auto-reset toggle via Blynk
BLYNK_WRITE(V15) {
  autoResetEnabled = (param.asInt() == 1);
  Serial.print("Auto-reset ");
  Serial.println(autoResetEnabled ? "enabled" : "disabled");
  Blynk.logEvent(autoResetEnabled ? "auto_reset_on" : "auto_reset_off");
}

// Synchronize states when Blynk connects
BLYNK_CONNECTED() {
  blynkConnected = true;
  Blynk.syncVirtual(V3);  // Sync the SD logging state
  Blynk.syncVirtual(V15); // Sync the auto-reset state
}

// Read battery voltage
float readBattery(uint8_t pin) {
  int vref = 1100;
  uint16_t volt = analogRead(pin);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

// Add these global variables
JsonDocument doc;
HardwareSerial ArduinoSerial(2); // Using UART2 for Arduino communication

// Process commands received from Arduino
void processIncomingCommands() {
  if (ArduinoSerial.available()) {
    // Wait for a complete JSON object
    String jsonString = "";
    bool foundStart = false;
    bool foundEnd = false;
    
    // Clear any garbage data first
    while (ArduinoSerial.available() && ArduinoSerial.peek() != '{') {
      ArduinoSerial.read();
    }
    
    // Read until we find a complete JSON object
    unsigned long startTime = millis();
    while (ArduinoSerial.available() && (millis() - startTime < 1000)) {
      char c = ArduinoSerial.read();
      
      if (c == '{') {
        foundStart = true;
        jsonString = "{";
        continue;
      }
      
      if (foundStart) {
        jsonString += c;
        
        if (c == '}') {
          foundEnd = true;
          break;
        }
      }
    }
    
    // Only process if we have a complete JSON object
    if (foundStart && foundEnd) {
      Serial.println("Received JSON from Arduino: " + jsonString);
      
      // Parse JSON
      JsonDocument cmdDoc;
      DeserializationError error = deserializeJson(cmdDoc, jsonString);
      
      if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
      }
      
      // Extract command
      String command = cmdDoc["cmd"].as<String>();
      
      // Update communication timestamp
      lastCommTime = millis();
      
      // Handle different commands
      if (command == CMD_RESET) {
        // Arduino is notifying about a reset
        Serial.println("Arduino has reset");
        sendCommand(CMD_ACK, "");
      }
      else if (command == CMD_DATA) {
        // Received sensor data
        JsonArray dataArray = cmdDoc["data"].as<JsonArray>();
        
        if (dataArray.size() >= 15) {
          // Extract values from array based on their position
          // time = dataArray[0]; // We don't need this on ESP32 side
          temperature = dataArray[1]; // temperature
          humidity = dataArray[2];    // humidity
          pressure = dataArray[3];    // pressure
          // altitude = dataArray[4]; // We don't need this on ESP32 side
          v_CO_w = dataArray[5];      // CO_w
          v_CO_a = dataArray[6];      // CO_a
          v_SO2_w = dataArray[7];     // SO2_w
          v_SO2_a = dataArray[8];     // SO2_a
          v_NO2_w = dataArray[9];     // NO2_w
          v_NO2_a = dataArray[10];    // NO2_a
          v_OX_w = dataArray[11];     // OX_w
          v_OX_a = dataArray[12];     // OX_a
          v_pid_w = dataArray[13];    // pid_w
          v_co2_w = dataArray[14];    // CO2_w
          
          Serial.println("Sensor data received successfully");
          sendCommand(CMD_COMPLETE, "");
        } else {
          Serial.println("Incomplete sensor data received");
          sendCommand(CMD_FAIL, "");
        }
      }
    }
  }
}

// Send a command to Arduino using JSON format
bool sendCommand(const char* cmd, const String& data, unsigned long timeout = CMD_TIMEOUT) {
  JsonDocument cmdDoc;
  cmdDoc["cmd"] = cmd;
  cmdDoc["data"] = data;
  
  String jsonString;
  serializeJson(cmdDoc, jsonString);
  
  Serial.print("Sending command to Arduino: ");
  Serial.println(jsonString);
  
  // Clear any pending data
  while (ArduinoSerial.available()) {
    ArduinoSerial.read();
  }
  
  // Send the command
  ArduinoSerial.println(jsonString);
  
  // Wait for response with timeout
  unsigned long startTime = millis();
  bool responseReceived = false;
  String response = "";
  
  while (millis() - startTime < timeout) {
    if (ArduinoSerial.available()) {
      // Look for JSON response
      String jsonResponse = "";
      bool foundStart = false;
      bool foundEnd = false;
      
      // Read until we find a complete JSON object
      unsigned long readStartTime = millis();
      while (ArduinoSerial.available() && (millis() - readStartTime < 1000)) {
        char c = ArduinoSerial.read();
        
        if (c == '{') {
          foundStart = true;
          jsonResponse = "{";
          continue;
        }
        
        if (foundStart) {
          jsonResponse += c;
          
          if (c == '}') {
            foundEnd = true;
            break;
          }
        }
      }
      
      if (foundStart && foundEnd) {
        JsonDocument respDoc;
        DeserializationError error = deserializeJson(respDoc, jsonResponse);
        
        if (!error) {
          response = respDoc["cmd"].as<String>();
          responseReceived = true;
          lastCommTime = millis(); // Update last successful communication time
          break;
        }
      }
    }
    delay(10);
  }
  
  if (responseReceived) {
    Serial.print("Received response: ");
    Serial.println(response);
    return true;
  } else {
    Serial.println("Command timeout!");
    return false;
  }
}

// Replace readSensorsFromArduino() with this new function
void readSensorsFromArduino() {
  // This is now handled in processIncomingCommands()
  // when JSON data is received from Arduino
}

// Command system implementation

// Send command with retry logic
bool sendCommandWithRetry(const char* cmd, String& response, int maxRetries) {
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    if (sendCommand(cmd, "", CMD_TIMEOUT)) {
      return true;
    }
    
    Serial.print("Retry ");
    Serial.print(attempt + 1);
    Serial.print(" of ");
    Serial.println(maxRetries);
    delay(200); // Short delay between retries
  }
  
  Serial.print("Failed to send command after ");
  Serial.print(maxRetries);
  Serial.println(" attempts");
  return false;
}

// Reset the ESP32
void resetSystem() {
  Serial.println("Performing system reset...");
  delay(500);
  ESP.restart();
}

// Check if Arduino accepts No Internet mode
bool checkNoInternetMode() {
  String response;
  if (sendCommand(CMD_NONET, "", CMD_TIMEOUT)) {
    return true;
  }
  
  Serial.println("Arduino did not accept No Internet mode");
  return false;
}

// Verify communication with Arduino with extended INFO command
bool verifyArduinoCommunication() {
  Serial.println("Attempting to establish communication with Arduino Mega...");
  
  if (sendCommand(CMD_INFO, "", CMD_TIMEOUT)) {
    Serial.println("Communication with Arduino Mega established successfully!");
    lastCommTime = millis();
    return true;
  }
  
  Serial.println("Failed to establish communication with Arduino Mega!");
  return false;
}

// Update sendSensorData() function
void sendSensorData() {
  // Send DATA command to Arduino to request fresh sensor data
  if (sendCommand(CMD_DATA, "", CMD_TIMEOUT)) {
    // Data will be received and processed in processIncomingCommands
    
    // Send to Blynk if connected
    if (internetAvailable && blynkConnected) {
      Blynk.virtualWrite(V0, temperature);
      Blynk.virtualWrite(V1, humidity);
      Blynk.virtualWrite(V2, ((readBattery(BAT_ADC) / 4200) * 100));
      
      // Gas sensors
      Blynk.virtualWrite(V4, v_CO_w);
      Blynk.virtualWrite(V5, v_CO_a);
      Blynk.virtualWrite(V6, v_SO2_w);
      Blynk.virtualWrite(V7, v_SO2_a);
      Blynk.virtualWrite(V8, v_NO2_w);
      Blynk.virtualWrite(V9, v_NO2_a);
      Blynk.virtualWrite(V10, v_OX_w);
      Blynk.virtualWrite(V11, v_OX_a);
      Blynk.virtualWrite(V12, v_pid_w);
      Blynk.virtualWrite(V13, v_co2_w);
    }
    
    // Log to SD if enabled
    if (!stopReading) {
      logToSD();
    }
  } else {
    Serial.println("Failed to get data from Arduino!");
    
    // Handle communication failure
    if (autoResetEnabled && millis() - lastCommTime > 60000) { // 1 minute without successful communication
      Serial.println("Communication lost for too long, resetting...");
      resetSystem();
    }
  }
}

// Set up system time from network
bool setupTimeWithRetry() {
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int min = 0;
  int sec = 0;
  float timezone = 0;
  
  // Try to get network time
  for (int8_t i = 5; i; i--) {
    Serial.println("Requesting current network time");
    if (modem.getNetworkTime(&year, &month, &day, &hour, &min, &sec, &timezone)) {
      Serial.printf("Date: %d-%02d-%02d Time: %02d:%02d:%02d\n", 
                    year, month, day, hour, min, sec);
      Serial.printf("Timezone: %.1f\n", timezone);
      
      // Set the system time
      setTime(hour, min, sec, day, month, year);
      return true;
    } else {
      Serial.println("Couldn't get network time, retrying in 10s.");
      delay(10000);
    }
  }
  return false;
}

// Setup SD card
void setupSD() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  
  Serial.println("SD card initialized");
  
  // Find available filename
  bool file = false;
  int count = 0;
  String baseFilename = "/DAT";
  while (!file) {
    String testFilename = baseFilename + String(count) + ".txt";
    if (SD.exists(testFilename)) {
      count++;
    } else {
      filename = testFilename;
      file = true;
    }
  }
  
  Serial.println("Using filename: " + filename);
  
  // Create header in the file and keep it open
  SDstorage = SD.open(filename.c_str(), FILE_WRITE);
  if (SDstorage) {
    SDstorage.println("Time Temperature(C) Humidity(%) Pressure(pa) CO_W CO_A SO2_W SO2_A NO2_W NO2_A OX_W OX_A PID_W CO2_W");
    SDstorage.flush();
  } else {
    Serial.println("Failed to open file for writing!");
  }
}

// Reconnect to SD card if needed
void reconnectSD() {
  while (!SD.begin(SD_CS)) {
    delay(500);
  }
}

// Setup the modem
void setupModem() {
  Serial1.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  
  // Setup modem pins
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300);
  digitalWrite(MODEM_PWRKEY, LOW);
  
  pinMode(MODEM_FLIGHT, OUTPUT);
  digitalWrite(MODEM_FLIGHT, HIGH);
  
  Serial.println("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, proceeding anyway");
  }
  
  String name = modem.getModemName();
  Serial.println("Modem Name: " + name);
}

// Log data to SD card
void logToSD() {
  if (!SD.exists(filename.c_str())) {
    reconnectSD();
    // Reopen the file if it was closed
    if (!SDstorage) {
      SDstorage = SD.open(filename.c_str(), FILE_APPEND);
      if (!SDstorage) {
        Serial.println("Failed to reopen file for appending!");
        return;
      }
    }
  }
  
  if (SDstorage) {
    // Get current date and time
    char timeStr[25];
    sprintf(timeStr, "%02d/%02d/%04d %02d:%02d:%02d", 
            day(), month(), year(), hour(), minute(), second());
    
    // Write data to file
    SDstorage.print(timeStr);
    SDstorage.print(" ");
    SDstorage.print(temperature, 4);
    SDstorage.print(" ");
    SDstorage.print(humidity, 4);
    SDstorage.print(" ");
    SDstorage.print(pressure, 4);
    SDstorage.print(" ");
    SDstorage.print(v_CO_w, 4);
    SDstorage.print(" ");
    SDstorage.print(v_CO_a, 4);
    SDstorage.print(" ");
    SDstorage.print(v_SO2_w, 4);
    SDstorage.print(" ");
    SDstorage.print(v_SO2_a, 4);
    SDstorage.print(" ");
    SDstorage.print(v_NO2_w, 4);
    SDstorage.print(" ");
    SDstorage.print(v_NO2_a, 4);
    SDstorage.print(" ");
    SDstorage.print(v_OX_w, 4);
    SDstorage.print(" ");
    SDstorage.print(v_OX_a, 4);
    SDstorage.print(" ");
    SDstorage.print(v_pid_w, 4);
    SDstorage.print(" ");
    SDstorage.println(v_co2_w, 4);
    
    // Flush the data to ensure it's written
    SDstorage.flush();
  } else {
    Serial.println("File not open for writing!");
  }
}

void setup() {
  // Initialize serial console
  Serial.begin(UART_BAUD);
  delay(1000);
  Serial.println("ESP32 Environmental Sensor System Starting...");
  
  // Initialize indicator LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize UART for Arduino communication
  ArduinoSerial.begin(115200, SERIAL_8N1, ARDUINO_UART_RX, ARDUINO_UART_TX);
  delay(500);
  
  // Verify communication with Arduino
  if (!verifyArduinoCommunication()) {
    Serial.println("System halted due to communication failure!");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED to indicate error
      delay(500);
    }
  }
  
  // Initialize modem
  setupModem();
  
  // Setup SD card
  setupSD();
  
  // Get and set time from network
  bool timeSet = false;
  for (int8_t i = 5; i; i--) {
    if (setupTimeWithRetry()) {
      timeSet = true;
      break;
    }
    delay(2000);
  }
  
  // Initialize Blynk with retry and fallback logic
  bool blynkInitialized = false;
  for (int8_t i = 10; i; i--) {
    Serial.print("Attempting to connect to Blynk (");
    Serial.print(i);
    Serial.println(" attempts left)...");
    
    Blynk.begin(auth, modem, apn, user, pass);
    
    // Wait for connection for 10 seconds
    for (int j = 0; j < 10; j++) {
      if (Blynk.connected()) {
        blynkInitialized = true;
        blynkConnected = true;
        Serial.println("Connected to Blynk!");
        break;
      }
      delay(1000);
    }
    
    if (blynkInitialized) {
      break;
    }
    
    delay(2000);
  }
  
  // If Blynk failed to initialize, check if Arduino accepts No Internet mode
  if (!blynkInitialized) {
    Serial.println("Failed to connect to Blynk after multiple attempts");
    
    if (checkNoInternetMode()) {
      internetAvailable = false;
      Serial.println("Continuing in No Internet mode");
    } else {
      Serial.println("Arduino did not accept No Internet mode, halting system");
      while (1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED to indicate error
        delay(200);
      }
    }
  }
  
  // Setup timers for data sending and command processing
  timer.setInterval(10000L, sendSensorData); // Every 10 seconds
  timer.setInterval(COMM_CHECK_MS, processIncomingCommands); // Check for incoming commands regularly
  
  Serial.println("Setup complete!");
}

void loop() {
  // Only run Blynk if internet is available
  if (internetAvailable && blynkConnected) {
    Blynk.run();
  }
  
  timer.run();
} 