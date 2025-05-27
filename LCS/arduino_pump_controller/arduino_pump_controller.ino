// Arduino Mega 2560 Pump Controller
// Receives JSON commands from ESP32 via Serial and controls air pumps and valve
// Supports duty cycle control for pump flow rate

#include <ArduinoJson.h> // Required for JSON parsing

// Pin Definitions for Arduino Mega
#define PUMP1_PIN 2    // Digital pin for Pump 1 control
#define PUMP2_PIN 3    // Digital pin for Pump 2 control  
#define VALVE_PIN 4    // Digital pin for Valve control

// PWM Configuration for Pumps (if using PWM-capable pins)
#define PUMP1_PWM_PIN 5    // PWM pin for Pump 1 speed control
#define PUMP2_PWM_PIN 6    // PWM pin for Pump 2 speed control

// Command Strings (received from ESP32 via JSON) - must match config.h
#define CMD_INFLATE "INFLATE"
#define CMD_DEFLATE "DEFLATE"
#define CMD_STOP "STOP"

// Serial Communication
#define SERIAL_BAUD 9600

void setup() {
  // Initialize Serial communication with ESP32
  Serial.begin(SERIAL_BAUD);
  
  // Initialize pump and valve pins
  pinMode(PUMP1_PIN, OUTPUT);
  pinMode(PUMP2_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  
  // Initialize PWM pins for speed control
  pinMode(PUMP1_PWM_PIN, OUTPUT);
  pinMode(PUMP2_PWM_PIN, OUTPUT);
  
  // Ensure everything is off initially
  stopAllPumpsAndValve();
}

void loop() {
  // Process incoming JSON commands
  processIncomingCommands();
  delay(10); // Small delay for stability
}

// Process incoming JSON commands
void processIncomingCommands() {
  if (Serial.available()) {
    String jsonString = "";
    bool foundStart = false;
    bool foundEnd = false;
    
    while (Serial.available()) {
      char c = Serial.read();
      if (!foundStart) {
        if (c == '{') { foundStart = true; jsonString = c; }
      } else {
        jsonString += c;
        if (c == '}') { foundEnd = true; break; }
      }
    }
    
    if (foundStart && foundEnd) {
      JsonDocument cmdDoc;
      DeserializationError error = deserializeJson(cmdDoc, jsonString);
      
      if (error) {
        return; // Just ignore bad JSON, no response needed
      }
      
      String command = cmdDoc["cmd"].as<String>();
      String data = cmdDoc["data"].as<String>();
      
      // Parse duty cycle from data field (0-255)
      int dutyCycle = data.toInt();
      if (dutyCycle <= 0) dutyCycle = 255; // Default to full speed
      
      if (command == CMD_INFLATE) {
        executeInflate(dutyCycle);
      } else if (command == CMD_DEFLATE) {
        executeDeflate(dutyCycle);
      } else if (command == CMD_STOP) {
        stopAllPumpsAndValve();
      }
      // No response needed - ESP32 doesn't wait for it
    }
  }
}

void executeInflate(int dutyCycle) {
  // Turn on both pumps with specified duty cycle for inflation
  digitalWrite(PUMP1_PIN, HIGH);
  digitalWrite(PUMP2_PIN, HIGH);
  analogWrite(PUMP1_PWM_PIN, dutyCycle);
  analogWrite(PUMP2_PWM_PIN, dutyCycle);
  // Turn on valve for inflation path
  digitalWrite(VALVE_PIN, HIGH);
}

void executeDeflate(int dutyCycle) {
  // Turn on both pumps with specified duty cycle for deflation
  digitalWrite(PUMP1_PIN, HIGH);
  digitalWrite(PUMP2_PIN, HIGH);
  analogWrite(PUMP1_PWM_PIN, dutyCycle);
  analogWrite(PUMP2_PWM_PIN, dutyCycle);
  // Turn off valve for deflation path
  digitalWrite(VALVE_PIN, LOW);
}

void stopAllPumpsAndValve() {
  // Turn off all pumps and valve
  digitalWrite(PUMP1_PIN, LOW);
  digitalWrite(PUMP2_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
  analogWrite(PUMP1_PWM_PIN, 0);
  analogWrite(PUMP2_PWM_PIN, 0);
} 