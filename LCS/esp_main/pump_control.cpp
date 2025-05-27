#include "global.h" // Include first for the macro definition

#ifdef ENABLE_PUMP_CONTROL

#include "pump_control.h"
#include "global.h"       // For ESP_BT extern declaration and Serial logging
#include "communication.h" // For processIncomingCommands
#include <ArduinoJson.h>

/**
 * @brief Initializes Bluetooth Serial for pump control communication.
 * @return True if initialization was successful, false otherwise.
 */
bool setupPumpControl() {
    Serial.println("Initializing Bluetooth Serial for Pump Control...");
    if (!ESP_BT.begin("ESP32_EnvSensor_BT")) { // Start Bluetooth with a name
      Serial.println("An error occurred initializing Pump Control Bluetooth");
      return false;
    } else {
      Serial.println("Pump Control Bluetooth initialized. Ready to pair with ESP32_PumpController_BT");
      Serial.println("Waiting for pump controller to connect...");
      return true;
    }
}

/**
 * @brief Handles incoming commands from the connected Bluetooth device.
 * Processes DATA requests from pump ESP32 and sends sensor data response.
 */
void handlePumpControlCommands() {
    if (ESP_BT.hasClient() && ESP_BT.available()) {
        String jsonString = "";
        bool foundStart = false;
        bool foundEnd = false;
        
        // Read complete JSON message
        while (ESP_BT.available()) {
            char c = ESP_BT.read();
            if (!foundStart) {
                if (c == '{') {
                    foundStart = true;
                    jsonString = c;
                }
            } else {
                jsonString += c;
                if (c == '}') {
                    foundEnd = true;
                    break;
                }
            }
        }
        
        if (foundStart && foundEnd) {
            Serial.print("Received from pump ESP32: ");
            Serial.println(jsonString);
            
            JsonDocument cmdDoc;
            DeserializationError error = deserializeJson(cmdDoc, jsonString);
            
            if (error) {
                Serial.print("JSON parsing failed: ");
                Serial.println(error.c_str());
                return;
            }
            
            String command = cmdDoc["cmd"].as<String>();
            
            if (command == CMD_DATA) {
                // Send sensor data to pump ESP32
                sendSensorDataToPump();
            } else if (command == CMD_INFO) {
                // Send ACK response for INFO command
                sendCommandWithoutResponse(ESP_BT, CMD_ACK, "");
            } else {
                Serial.print("Unknown command from pump ESP32: ");
                Serial.println(command);
            }
        } else if (foundStart && !foundEnd) {
            Serial.println("Incomplete JSON received from pump ESP32. Discarding.");
        }
    }
}

/**
 * @brief Sends current sensor data to pump ESP32 via Bluetooth
 */
void sendSensorDataToPump() {
    if (!ESP_BT.hasClient()) {
        Serial.println("No Bluetooth client connected for pump data");
        return;
    }
    
    // Create sensor data array matching the format expected by pump ESP32
    JsonDocument responseDoc;
    responseDoc["cmd"] = CMD_DATA;
    
    JsonArray dataArray = responseDoc["data"].to<JsonArray>();
    dataArray.add(millis());        // 0: timestamp
    dataArray.add(temperature);     // 1: temperature
    dataArray.add(humidity);        // 2: humidity  
    dataArray.add(pressure);        // 3: pressure
    dataArray.add(0.0);            // 4: battery (placeholder)
    dataArray.add(v_CO_w);         // 5: CO working
    dataArray.add(v_CO_a);         // 6: CO auxiliary
    dataArray.add(v_SO2_w);        // 7: SO2 working
    dataArray.add(v_SO2_a);        // 8: SO2 auxiliary
    dataArray.add(v_NO2_w);        // 9: NO2 working
    dataArray.add(v_NO2_a);        // 10: NO2 auxiliary
    dataArray.add(v_OX_w);         // 11: OX working
    dataArray.add(v_OX_a);         // 12: OX auxiliary
    dataArray.add(v_pid_w);        // 13: VOC (PID) - this is what pump ESP32 uses for auto-trigger
    dataArray.add(v_co2_w);        // 14: CO2
    
    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);
    
    ESP_BT.println(jsonResponse);
    
    Serial.print("Sent sensor data to pump ESP32: ");
    Serial.println(jsonResponse);
}

#endif // ENABLE_PUMP_CONTROL
