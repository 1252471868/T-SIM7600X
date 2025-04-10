/*
  Arduino Mega Command System for ESP32 Communication
  
  This implements the Arduino-side command handlers for the ESP32 communication system.
  It should be integrated with the main ArduinoMega.ino file.
  
  Commands:
  - INFO: Communication verification
  - DATA: Request for sensor data
  - RESET: Reset request
  - NONET: Query for No Internet Mode acceptance
*/

// Command definitions - must match ESP32 side
#define CMD_INFO        "INFO"     // Check communication status
#define CMD_DATA        "DATA"     // Request sensor data
#define CMD_RESET       "RESET"    // Reset command
#define CMD_ACK         "ACK"      // Acknowledgment
#define CMD_NONET       "NONET"    // No internet mode query
#define CMD_YES         "YES"      // Positive response
#define CMD_NO          "NO"       // Negative response
#define CMD_COMPLETE    "COMPLETE" // Data successfully received
#define CMD_FAIL        "FAIL"     // Command failed

// Communication control
#define COMM_TIMEOUT    3000       // Timeout for ESP32 communication
unsigned long lastEspResponseTime = 0;
bool autoResetEnabled = true;
const int resetPin = 8;  // Digital pin connected to reset circuit (adjust as needed)

// Process commands received from ESP32
void processEspCommands() {
  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.print("Received command from ESP32: ");
      Serial.println(command);
      
      // Handle different commands
      if (command == CMD_INFO) {
        // Information/communication check
        Serial2.println(CMD_ACK);
        lastEspResponseTime = millis();
      }
      else if (command == CMD_DATA) {
        // ESP32 is requesting sensor data
        Serial2.println(CMD_ACK);
        sendSensorDataToEsp();
        lastEspResponseTime = millis();
      }
      else if (command == CMD_RESET) {
        // ESP32 is requesting a reset
        Serial2.println(CMD_ACK);
        Serial.println("Reset requested by ESP32");
        delay(500);
        resetArduino();
      }
      else if (command == CMD_NONET) {
        // ESP32 is asking if we accept No Internet mode
        Serial2.println(CMD_YES);  // We accept No Internet mode by default
        Serial.println("Accepting No Internet mode");
        lastEspResponseTime = millis();
      }
      else if (command == CMD_COMPLETE) {
        // ESP32 has successfully received data
        Serial.println("ESP32 confirmed data reception");
        lastEspResponseTime = millis();
      }
    }
  }
  
  // Check for ESP32 communication timeout
  if (autoResetEnabled && millis() - lastEspResponseTime > 120000) { // 2 minutes without response
    Serial.println("ESP32 communication timeout, resetting...");
    resetArduino();
  }
}

// Send sensor data to ESP32 in JSON format
void sendSensorDataToEsp() {
  // Create JSON document
  StaticJsonDocument<512> doc;
  
  // Add sensor readings to the document
  doc["temperature"] = bme.temperature;
  doc["humidity"] = bme.humidity;
  doc["pressure"] = bme.pressure / 100.0;
  doc["CO_w"] = analogRead(pin_CO_w) / 1024.000 * voltageSupply;
  doc["CO_a"] = analogRead(pin_CO_a) / 1024.000 * voltageSupply;
  doc["SO2_w"] = analogRead(pin_SO2_w) / 1024.000 * voltageSupply;
  doc["SO2_a"] = analogRead(pin_SO2_a) / 1024.000 * voltageSupply;
  doc["NO2_w"] = analogRead(pin_NO2_w) / 1024.000 * voltageSupply;
  doc["NO2_a"] = analogRead(pin_NO2_a) / 1024.000 * voltageSupply;
  doc["OX_w"] = analogRead(pin_OX_w) / 1024.000 * voltageSupply;
  doc["OX_a"] = analogRead(pin_OX_a) / 1024.000 * voltageSupply;
  doc["pid_w"] = analogRead(pin_pid) / 1024.000 * voltageSupply;
  doc["CO2_w"] = analogRead(pin_CO2) / 1024.000 * voltageSupply;
  
  // Serialize JSON to Serial2
  serializeJson(doc, Serial2);
  Serial2.println(); // Add newline
  
  Serial.println("Sensor data sent to ESP32");
}

// Reset the Arduino
void resetArduino() {
  Serial.println("Resetting Arduino...");
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  // If hardware reset isn't possible, we'll reach here
  // and can use software reset as backup
  asm volatile ("jmp 0");  // Software reset
}

// Setup function to be called from main setup()
void setupCommandSystem() {
  Serial.println("Initializing command system...");
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
  lastEspResponseTime = millis(); // Initialize timestamp
}

// This should be called from the main loop
void loopCommandSystem() {
  processEspCommands();
} 