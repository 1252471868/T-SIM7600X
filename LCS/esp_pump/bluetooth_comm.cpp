#include "bluetooth_comm.h"
#include "global.h"
#include <ArduinoJson.h>

// Local variables for VOC data storage
static float latestVOCReading = 0.0;
static bool hasValidVOCData = false;

/**
 * @brief Initializes Bluetooth Serial communication with main ESP32
 * @return true if initialization successful, false otherwise
 */
bool initializeBluetoothComm() {
    Serial.println("Initializing Bluetooth communication with main ESP32...");
    
    if (!MainESP32_BT.begin(BT_DEVICE_NAME)) {
        Serial.println("An error occurred initializing Bluetooth");
        return false;
    }
    
    Serial.print("Bluetooth initialized as: ");
    Serial.println(BT_DEVICE_NAME);
    Serial.print("Ready to connect to: ");
    Serial.println(MAIN_ESP32_BT_NAME);
    
    return true;
}

/**
 * @brief Requests VOC data from main ESP32 by sending DATA command
 */
void requestVOCData() {
    if (!MainESP32_BT.hasClient()) {
        if (!mainESP32Connected) {
            Serial.println("No Bluetooth client connected to main ESP32");
        }
        mainESP32Connected = false;
        return;
    }
    
    // Send DATA command to request sensor data
    JsonDocument cmdDoc;
    cmdDoc["cmd"] = CMD_DATA;
    cmdDoc["data"] = "";
    
    String jsonCommand;
    serializeJson(cmdDoc, jsonCommand);
    
    MainESP32_BT.println(jsonCommand);
    
    Serial.print("Requested VOC data from main ESP32: ");
    Serial.println(jsonCommand);
    
    lastMainESP32Comm = millis();
}

/**
 * @brief Processes incoming commands from main ESP32 via Bluetooth
 */
void processMainESP32Commands() {
    if (!MainESP32_BT.hasClient()) {
        return;
    }
    
    if (MainESP32_BT.available()) {
        String jsonString = "";
        bool foundStart = false;
        bool foundEnd = false;
        
        // Read complete JSON message
        while (MainESP32_BT.available()) {
            char c = MainESP32_BT.read();
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
            Serial.print("Received from main ESP32: ");
            Serial.println(jsonString);
            
            JsonDocument responseDoc;
            DeserializationError error = deserializeJson(responseDoc, jsonString);
            
            if (error) {
                Serial.print("JSON parsing failed: ");
                Serial.println(error.c_str());
                return;
            }
            
            String command = responseDoc["cmd"].as<String>();
            lastMainESP32Comm = millis();
            mainESP32Connected = true;
            
            if (command == CMD_DATA) {
                // Process sensor data response
                JsonArray dataArray = responseDoc["data"].as<JsonArray>();
                if (dataArray.size() >= 14) {
                    // VOC data is at index 13 (v_pid_w in main ESP32)
                    latestVOCReading = dataArray[13].as<float>();
                    hasValidVOCData = true;
                    
                    Serial.print("Received VOC reading: ");
                    Serial.println(latestVOCReading);
                    
                } else {
                    Serial.print("Incomplete sensor data received. Expected >=14 items, got: ");
                    Serial.println(dataArray.size());
                }
            } else if (command == CMD_ACK) {
                Serial.println("Received ACK from main ESP32");
            } else {
                Serial.print("Unknown command from main ESP32: ");
                Serial.println(command);
            }
        } else if (foundStart && !foundEnd) {
            Serial.println("Incomplete JSON received from main ESP32. Discarding.");
        }
    }
}

/**
 * @brief Verifies connection with main ESP32 by sending INFO command
 * @return true if main ESP32 responds, false otherwise
 */
bool verifyMainESP32Connection() {
    if (!MainESP32_BT.hasClient()) {
        Serial.println("No Bluetooth client connected");
        return false;
    }
    
    Serial.println("Verifying connection with main ESP32...");
    
    // Send INFO command
    JsonDocument cmdDoc;
    cmdDoc["cmd"] = CMD_INFO;
    cmdDoc["data"] = "";
    
    String jsonCommand;
    serializeJson(cmdDoc, jsonCommand);
    
    MainESP32_BT.println(jsonCommand);
    Serial.print("Sent INFO command: ");
    Serial.println(jsonCommand);
    
    // Wait for response
    unsigned long startTime = millis();
    while (millis() - startTime < CMD_TIMEOUT) {
        if (MainESP32_BT.available()) {
            String response = MainESP32_BT.readStringUntil('\n');
            if (response.indexOf("ACK") >= 0) {
                mainESP32Connected = true;
                lastMainESP32Comm = millis();
                Serial.println("Main ESP32 connection verified");
                return true;
            }
        }
        delay(10);
    }
    
    Serial.println("Main ESP32 connection verification failed");
    mainESP32Connected = false;
    return false;
}

/**
 * @brief Gets the latest VOC reading received from main ESP32
 * @return Latest VOC concentration value
 */
float getLatestVOCReading() {
    if (!hasValidVOCData) {
        Serial.println("Warning: No valid VOC data available yet");
        return 0.0;
    }
    return latestVOCReading;
} 