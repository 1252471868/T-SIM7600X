// This sketch is for directly testing the pump and valve hardware outputs.
// It bypasses Bluetooth and allows direct control via Serial Monitor.

#include <Arduino.h> // Core Arduino functions and macros
#include "../esp_pump/config.h" // Include config from the parent esp_pump directory

void printInstructions() {
  Serial.println(F("\n--- Pump Hardware Test --- "));
  Serial.println(F("Enter commands via Serial Monitor:"));
  Serial.println(F("  1: Pump 1 ON"));
  Serial.println(F("  q: Pump 1 OFF"));
  Serial.println(F("  2: Pump 2 ON"));
  Serial.println(F("  w: Pump 2 OFF"));
  Serial.println(F("  v: Valve ON"));
  Serial.println(F("  c: Valve OFF"));
  Serial.println(F("  i: Simulate INFLATE (Pumps ON, Valve ON)"));
  Serial.println(F("  d: Simulate DEFLATE (Pumps ON, Valve OFF)"));
  Serial.println(F("  s: STOP ALL (Pumps OFF, Valve OFF)"));
  Serial.println(F("  h: Print this help menu"));
  Serial.println(F("---------------------------"));
}

void stopAllOutputs() {
  Serial.println(F("Stopping all outputs."));
  ledcWrite(PUMP_PWM_CHANNEL_1, 0);
  ledcWrite(PUMP_PWM_CHANNEL_2, 0);
  digitalWrite(VALVE_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for serial connection

  Serial.println(F("ESP32 Pump Hardware Test Sketch Initializing..."));

  // Setup Pump 1 PWM
  Serial.print(F("Setting up Pump 1 (Pin: ")); Serial.print(PUMP1_PIN); 
  Serial.print(F(", Channel: ")); Serial.print(PUMP_PWM_CHANNEL_1); Serial.println(F(")"));
  ledcSetup(PUMP_PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PUMP1_PIN, PUMP_PWM_CHANNEL_1);

  // Setup Pump 2 PWM
  Serial.print(F("Setting up Pump 2 (Pin: ")); Serial.print(PUMP2_PIN); 
  Serial.print(F(", Channel: ")); Serial.print(PUMP_PWM_CHANNEL_2); Serial.println(F(")"));
  ledcSetup(PUMP_PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PUMP2_PIN, PUMP_PWM_CHANNEL_2);

  // Setup Valve Pin
  Serial.print(F("Setting up Valve (Pin: ")); Serial.print(VALVE_PIN); Serial.println(F(")"));
  pinMode(VALVE_PIN, OUTPUT);

  // Ensure everything is off initially
  stopAllOutputs();

  printInstructions();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.println(); // Newline for cleaner output

    switch (cmd) {
      case '1':
        Serial.println(F("Pump 1 ON"));
        ledcWrite(PUMP_PWM_CHANNEL_1, PUMP_DUTY_CYCLE);
        break;
      case 'q':
      case 'Q':
        Serial.println(F("Pump 1 OFF"));
        ledcWrite(PUMP_PWM_CHANNEL_1, 0);
        break;
      case '2':
        Serial.println(F("Pump 2 ON"));
        ledcWrite(PUMP_PWM_CHANNEL_2, PUMP_DUTY_CYCLE);
        break;
      case 'w':
      case 'W':
        Serial.println(F("Pump 2 OFF"));
        ledcWrite(PUMP_PWM_CHANNEL_2, 0);
        break;
      case 'v':
      case 'V':
        Serial.println(F("Valve ON"));
        digitalWrite(VALVE_PIN, HIGH);
        break;
      case 'c':
      case 'C':
        Serial.println(F("Valve OFF"));
        digitalWrite(VALVE_PIN, LOW);
        break;
      case 'i':
      case 'I':
        Serial.println(F("Simulating INFLATE: Pumps ON, Valve ON"));
        ledcWrite(PUMP_PWM_CHANNEL_1, PUMP_DUTY_CYCLE);
        ledcWrite(PUMP_PWM_CHANNEL_2, PUMP_DUTY_CYCLE);
        digitalWrite(VALVE_PIN, HIGH);
        break;
      case 'd':
      case 'D':
        Serial.println(F("Simulating DEFLATE: Pumps ON, Valve OFF"));
        ledcWrite(PUMP_PWM_CHANNEL_1, PUMP_DUTY_CYCLE);
        ledcWrite(PUMP_PWM_CHANNEL_2, PUMP_DUTY_CYCLE);
        digitalWrite(VALVE_PIN, LOW);
        break;
      case 's':
      case 'S':
        stopAllOutputs();
        break;
      case 'h':
      case 'H':
        printInstructions();
        break;
      default:
        if (cmd != '\n' && cmd != '\r') { // Ignore newline/carriage return
            Serial.print(F("Unknown command: "));
            Serial.println(cmd);
            Serial.println(F("Type 'h' for help."));
        }
        break;
    }
    // Clear any remaining characters in the serial buffer for this command
    while(Serial.available()) { Serial.read(); }
  }
  delay(50); // Small delay
} 