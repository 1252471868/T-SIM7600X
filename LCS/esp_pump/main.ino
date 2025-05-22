#include "config.h"
#include "BluetoothSerial.h"
#include <ArduinoJson.h> // Required for JSON parsing

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial ESP_BT_PUMP;

unsigned long pumpStartTime = 0;
bool pumping = false;
// String currentPumpOperation = ""; // To track if inflating or deflating for timed stop logic if needed beyond just 'pumping'

// Helper function to stop all pumps and the valve
void stopAllPumpsAndValve() {
  Serial.println(F("Stopping all pumps and valve."));
  ledcWrite(PUMP_PWM_CHANNEL_1, 0);
  ledcWrite(PUMP_PWM_CHANNEL_2, 0);
  digitalWrite(VALVE_PIN, LOW);
  pumping = false;
  // currentPumpOperation = "";
}

// Helper function to send a JSON response via Bluetooth
void sendPumpJsonResponse(const String& cmd, const String& data) {
  DynamicJsonDocument doc(128); // Adjust size as needed for response
  doc["cmd"] = cmd;
  doc["data"] = data;
  
  String jsonResponse;
  serializeJson(doc, jsonResponse);
  ESP_BT_PUMP.println(jsonResponse);
  Serial.print(F("Sent BT Response: "));
  Serial.println(jsonResponse);
}

// Processes incoming commands from Bluetooth
void processBluetoothCommands() {
  if (ESP_BT_PUMP.available()) {
    String payload = ESP_BT_PUMP.readStringUntil('\n');
    payload.trim();
    Serial.print(F("Received BT payload: "));
    Serial.println(payload);

    if (payload.length() == 0) return; // Ignore empty payloads

    DynamicJsonDocument doc(256); // Adjust size as needed for commands
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      sendPumpJsonResponse(CMD_NACK, "JSON_PARSE_ERROR");
      return;
    }

    const char* command_char = doc["cmd"];
    if (command_char == nullptr) {
        Serial.println(F("Command is null in JSON"));
        sendPumpJsonResponse(CMD_NACK, "MISSING_CMD_FIELD");
        return;
    }
    String command = String(command_char);

    if (command.equals(CMD_PUMP_INFLATE)) {
      Serial.println(F("Executing INFLATE command."));
      ledcWrite(PUMP_PWM_CHANNEL_1, PUMP_DUTY_CYCLE);
      ledcWrite(PUMP_PWM_CHANNEL_2, PUMP_DUTY_CYCLE);
      digitalWrite(VALVE_PIN, HIGH); // Valve ON for inflation
      pumpStartTime = millis();
      pumping = true;
      // currentPumpOperation = CMD_PUMP_INFLATE;
      sendPumpJsonResponse(CMD_ACK, CMD_PUMP_INFLATE);
    } else if (command.equals(CMD_PUMP_DEFLATE)) {
      Serial.println(F("Executing DEFLATE command."));
      ledcWrite(PUMP_PWM_CHANNEL_1, PUMP_DUTY_CYCLE);
      ledcWrite(PUMP_PWM_CHANNEL_2, PUMP_DUTY_CYCLE);
      digitalWrite(VALVE_PIN, LOW); // Valve OFF for deflation
      pumpStartTime = millis();
      pumping = true;
      // currentPumpOperation = CMD_PUMP_DEFLATE;
      sendPumpJsonResponse(CMD_ACK, CMD_PUMP_DEFLATE);
    } else if (command.equals(CMD_PUMP_STOP)) {
      Serial.println(F("Executing STOP command."));
      stopAllPumpsAndValve();
      sendPumpJsonResponse(CMD_ACK, CMD_PUMP_STOP);
    } else {
      Serial.print(F("Unknown command in JSON: "));
      Serial.println(command);
      sendPumpJsonResponse(CMD_NACK, "UNKNOWN_COMMAND");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("ESP32 Pump Controller Initializing..."));

  // Setup Pump 1 PWM
  ledcSetup(PUMP_PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PUMP1_PIN, PUMP_PWM_CHANNEL_1);

  // Setup Pump 2 PWM
  ledcSetup(PUMP_PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PUMP2_PIN, PUMP_PWM_CHANNEL_2);

  // Setup Valve Pin
  pinMode(VALVE_PIN, OUTPUT);

  // Ensure everything is off initially
  stopAllPumpsAndValve();

  // Initialize Bluetooth
  if (!ESP_BT_PUMP.begin(PUMP_BT_DEVICE_NAME)) {
    Serial.println(F("An error occurred initializing Bluetooth for pump controller"));
  } else {
    Serial.print(F("Bluetooth Pump Controller Initialized: "));
    Serial.println(PUMP_BT_DEVICE_NAME);
    Serial.println(F("Ready to pair and receive JSON commands."));
  }
}

void loop() {
  processBluetoothCommands();

  if (pumping && (millis() - pumpStartTime >= PUMP_DURATION_MS)) {
    Serial.println(F("Pump duration reached. Stopping pumps."));
    stopAllPumpsAndValve();
  }
  
  delay(10); // Small delay
}
