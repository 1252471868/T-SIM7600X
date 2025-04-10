/*
  Reference Command System Implementation for Arduino Mega
  To be integrated into sniffbox_wifi.ino
*/

/****************** ADD THESE AT THE TOP WITH OTHER INCLUDES ******************/
// Command definitions for ESP32 communication
#define CMD_INFO        "INFO"     // Check communication status
#define CMD_DATA        "DATA"     // Request sensor data
#define CMD_RESET       "RESET"    // Reset command
#define CMD_ACK         "ACK"      // Acknowledgment
#define CMD_NONET       "NONET"    // No internet mode query
#define CMD_YES         "YES"      // Positive response
#define CMD_NO          "NO"       // Negative response
#define CMD_COMPLETE    "COMPLETE" // Data successfully received
#define CMD_FAIL        "FAIL"     // Command failed

/****************** ADD THESE AFTER OTHER GLOBAL VARIABLES ******************/
// Communication control
unsigned long lastEspCommandTime = 0;   // Last time we received a command from ESP32
unsigned long lastCommunicationCheck = 0; // Last time we checked communication
const unsigned long COMM_TIMEOUT = 60000; // 60 seconds timeout
bool autoResetEnabled = true;
const int resetPin = 8;  // Digital pin connected to reset circuit (adjust as needed)

/****************** ADD THESE FUNCTION IMPLEMENTATIONS ******************/

// Process commands received from ESP32
void processEspCommands() {
  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.print("Received command from ESP32: ");
      Serial.println(command);
      
      // Update communication timestamp
      lastEspCommandTime = millis();
      
      // Handle different commands
      if (command == CMD_INFO) {
        // Information/communication check
        Serial2.println(CMD_ACK);
        Serial.println("Info request acknowledged");
      }
      else if (command == CMD_DATA) {
        // ESP32 is requesting sensor data
        Serial2.println(CMD_ACK);
        sendSensorDataToEsp();
      }
      else if (command == CMD_RESET) {
        // ESP32 is requesting a reset
        Serial2.println(CMD_ACK);
        Serial.println("Reset requested by ESP32");
        delay(200);
        resetArduino();
      }
      else if (command == CMD_NONET) {
        // ESP32 is asking if we accept No Internet mode
        Serial2.println(CMD_YES);  // We accept No Internet mode by default
        Serial.println("Accepting No Internet mode");
      }
      else if (command == CMD_COMPLETE) {
        // ESP32 has successfully received data
        Serial.println("ESP32 confirmed data reception");
      }
    }
  }
  
  // Check for ESP32 communication timeout every 5 seconds
  if (millis() - lastCommunicationCheck > 5000) {
    lastCommunicationCheck = millis();
    
    if (autoResetEnabled && (millis() - lastEspCommandTime > COMM_TIMEOUT)) {
      Serial.println("ESP32 communication timeout, resetting...");
      resetArduino();
    }
  }
}

// Reset the Arduino
void resetArduino() {
  Serial.println("Resetting Arduino...");
  
  // Try to notify ESP32 before reset
  Serial2.println("RESET");
  
  delay(100);
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  
  // If hardware reset doesn't work, use software reset
  asm volatile ("jmp 0");  // Software reset
}

// Send sensor data to ESP32 in JSON format
void sendSensorDataToEsp() {
  // Read sensor data
  double v_CO_w = analogRead(pin_CO_w) / 1024.000 * voltageSupply;
  double v_CO_a = analogRead(pin_CO_a) / 1024.000 * voltageSupply;
  double v_SO2_w = analogRead(pin_SO2_w) / 1024.000 * voltageSupply;
  double v_SO2_a = analogRead(pin_SO2_a) / 1024.000 * voltageSupply;
  double v_NO2_w = analogRead(pin_NO2_w) / 1024.000 * voltageSupply;
  double v_NO2_a = analogRead(pin_NO2_a) / 1024.000 * voltageSupply;
  double v_OX_w = analogRead(pin_OX_w) / 1024.000 * voltageSupply;
  double v_OX_a = analogRead(pin_OX_a) / 1024.000 * voltageSupply;
  double v_pid_w = analogRead(pin_pid) / 1024.000 * voltageSupply;
  double v_co2_w = analogRead(pin_CO2) / 1024.000 * voltageSupply;

  // Get BME680 readings
  if (!bme.performReading()) {
    Serial.println("Failed to perform BME680 reading");
    return;
  }
  
  // Create JSON document - reuse existing doc object
  doc.clear();
  
  // Format time string
  char timebuffer_save[25];
  sprintf(timebuffer_save, "%02d %02d %02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  
  // Add all data to the document
  doc["time"] = timebuffer_save;
  doc["temperature"] = bme.temperature;
  doc["humidity"] = bme.humidity;
  doc["pressure"] = bme.pressure;
  doc["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
  doc["CO_w"] = v_CO_w;
  doc["CO_a"] = v_CO_a;
  doc["SO2_w"] = v_SO2_w;
  doc["SO2_a"] = v_SO2_a;
  doc["NO2_w"] = v_NO2_w;
  doc["NO2_a"] = v_NO2_a;
  doc["OX_w"] = v_OX_w;
  doc["OX_a"] = v_OX_a;
  doc["pid_w"] = v_pid_w;
  doc["CO2_w"] = v_co2_w;
  
  // Serialize JSON to Serial2
  jsonData = "";
  serializeJson(doc, jsonData);
  Serial2.println(jsonData);
  
  Serial.println("Sensor data sent to ESP32");
}

// Check communication with ESP32 on startup - replace existing verification
void checkEspCommunication() {
  bool communicationEstablished = false;
  unsigned long startTime = millis();
  Serial.println("Checking ESP32 communication...");
  
  // Send reset notification
  Serial2.println("RESET");
  
  // Try to establish communication for up to 60 seconds
  while (millis() - startTime < 60000) {
    if (Serial2.available()) {
      String response = Serial2.readStringUntil('\n');
      response.trim();
      
      if (response == CMD_ACK || response == CMD_INFO) {
        Serial2.println(CMD_ACK);
        communicationEstablished = true;
        Serial.println("Communication with ESP32 established!");
        lcd.setCursor(0, 1);
        lcd.print("ESP32 Connected");
        lastEspCommandTime = millis();
        break;
      }
    }
    
    // Periodically send reset notification
    if (millis() - startTime > 5000 && (millis() - startTime) % 5000 < 100) {
      Serial2.println("RESET");
    }
    
    delay(100);
  }
  
  if (!communicationEstablished) {
    Serial.println("Failed to establish communication with ESP32, resetting...");
    lcd.setCursor(0, 1);
    lcd.print("ESP32 Failed!");
    delay(3000);
    resetArduino();
  }
}

/****************** SETUP CHANGES ******************/
/*
In your setup() function, add these lines:

  // Initialize reset pin
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
  
  // Update communication timestamp
  lastEspCommandTime = millis();

Replace the existing verification code with:
  checkEspCommunication();
*/

/****************** LOOP CHANGES ******************/
/*
At the beginning of your loop() function, add:

  // Process any incoming commands from ESP32
  processEspCommands();

Also add this line inside your LCD display loop to maintain responsiveness:
    // Process ESP32 commands while in display loop
    processEspCommands();

Remove or comment out the original JSON sending code at the end of the loop,
as this will now be handled by the sendSensorDataToEsp() function when
requested by the ESP32 through the DATA command.
*/ 