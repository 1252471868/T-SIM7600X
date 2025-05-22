#include "communication.h"
#include "global.h"
// #include <ArduinoJson.h> // Redundant: Included via communication.h
// #include <SD.h> // Needed for SD status check in sendSensorData (defined elsewhere)
// #include "sd_card.h" // Needed for logToSD call in sendSensorData (defined elsewhere)
// #include "sensors.h" // Needed for readBattery call in sendSensorData (defined elsewhere)
// #include "system_utils.h" // Added for resetSystem call in sendSensorData (defined elsewhere)

// Assume LED_PIN, BAT_ADC, VPIN_*, SD_CS are defined in EnvSensor.h
// Assume blynkConnected, internetAvailable, stopReading, autoResetEnabled are globals defined elsewhere
/**
 * @brief Processes incoming commands from a specified Stream.
 * Reads serial data, looks for a complete JSON object, parses it,
 * and acts based on the command received (e.g., RESET, DATA).
 * Sends ACK response back to the same Stream.
 * 
 * @param inputPort The Stream object to read commands from and send ACKs to.
 */
void processIncomingCommands(Stream& inputPort) { 
    if (inputPort.available()) {
        String jsonString = "";
        bool foundStart = false;
        bool foundEnd = false;
        unsigned long readStartTime = millis();
        while (inputPort.available()) { 
            char c = inputPort.read(); 
            if (!foundStart) {
                if (c == '{') { foundStart = true; jsonString = c; }
            } else {
                jsonString += c;
                if (c == '}') { foundEnd = true; break; }
            }
        }
        if (foundStart && foundEnd) {
            Serial.print("Received potential JSON: "); Serial.println(jsonString); 
            JsonDocument cmdDoc; 
            DeserializationError error = deserializeJson(cmdDoc, jsonString);
            if (error) {
                Serial.print("JSON parsing failed: "); Serial.println(error.c_str());
                return; 
            }
            String command = cmdDoc["cmd"].as<String>();
            lastCommTime = millis(); // Updates variable defined in this file
            Serial.print("Received command: "); Serial.println(command);
            if (command == CMD_RESET) {
                Serial.println("Device has reset. Sending ACK.");
                sendCommandWithoutResponse(inputPort, CMD_ACK, ""); // Send ACK back to input port
            }
            else if (command == CMD_DATA) {
                Serial.println("Processing sensor data...");
                JsonArray dataArray = cmdDoc["data"].as<JsonArray>();
                if (dataArray.size() >= 15) {
                    // Updates variables defined in this file
                    temperature = dataArray[1]; humidity = dataArray[2]; pressure = dataArray[3];
                    v_CO_w = dataArray[5]; v_CO_a = dataArray[6]; v_SO2_w = dataArray[7]; v_SO2_a = dataArray[8];
                    v_NO2_w = dataArray[9]; v_NO2_a = dataArray[10]; v_OX_w = dataArray[11]; v_OX_a = dataArray[12];
                    v_pid_w = dataArray[13]; v_co2_w = dataArray[14];
                    Serial.println("Sensor data received successfully. Sending ACK.");
                    sendCommandWithoutResponse(inputPort, CMD_ACK, ""); // Send ACK back to input port                 
                    sendSensorData();
                } else {
                    Serial.print("Incomplete sensor data received. Expected >=15 items, got: "); Serial.println(dataArray.size());
                    sendCommandWithoutResponse(inputPort, CMD_ACK, ""); // Send ACK back to input port
                }
            } else {
                 Serial.print("Unknown command received: "); Serial.println(command);
            }
        } else if (foundStart && !foundEnd) {
           Serial.println("Incomplete JSON received (no closing '}'). Discarding.");
        }
    }
}

/**
 * @brief Sends a command to the specified HardwareSerial port without waiting for a response.
 * Formats the command and data into a JSON string and sends it.
 *
 * @param serialPort The HardwareSerial port to send the command to.
 * @param cmd The command string (e.g., CMD_ACK).
 * @param data The data string associated with the command (can be empty).
 */
void sendCommandWithoutResponse(Stream& serialPort, const char* cmd, const String& data) {
    JsonDocument cmdDoc;
    cmdDoc["cmd"] = cmd;
    cmdDoc["data"] = data;
    String jsonString;
    serializeJson(cmdDoc, jsonString);
    serialPort.println(jsonString); 
    Serial.print("Sent command (no response wait): "); Serial.println(jsonString);
}

/**
 * @brief Sends a command to the specified HardwareSerial port and waits for a JSON response.
 * Formats the command and data into JSON, sends it, and waits
 * for a JSON response containing a "cmd" field within the specified timeout.
 *
 * @param serialPort The HardwareSerial port to send the command to and receive response from.
 * @param cmd The command string to send.
 * @param data The data string associated with the command.
 * @param timeout The maximum time in milliseconds to wait for a response.
 * @return true If a valid JSON response with a "cmd" field was received within the timeout.
 * @return false If the timeout occurred or JSON parsing failed.
 */
bool sendCommand(Stream& serialPort, const char* cmd, const String& data, unsigned long timeout) {
    JsonDocument cmdDoc;
    cmdDoc["cmd"] = cmd;
    cmdDoc["data"] = data;
    String jsonStringToSend;
    serializeJson(cmdDoc, jsonStringToSend);
    Serial.print("Sending command (expect response): "); Serial.println(jsonStringToSend);
    serialPort.println(jsonStringToSend); 
    unsigned long startTime = millis();
    while (millis() - startTime < timeout) {
        if (serialPort.available()) { 
            String jsonResponseString = "";
            bool foundRespStart = false, foundRespEnd = false;
            while (serialPort.available()) { 
                char c = serialPort.read(); 
                if (!foundRespStart) {
                    if (c == '{') { foundRespStart = true; jsonResponseString = c; }
                } else {
                    jsonResponseString += c;
                    if (c == '}') { foundRespEnd = true; break; }
                }
            }
            if (foundRespStart && foundRespEnd) {
                Serial.println("Received potential JSON response: " + jsonResponseString);
                JsonDocument respDoc; 
                DeserializationError error = deserializeJson(respDoc, jsonResponseString);
                if (!error && respDoc.containsKey("cmd")) { 
                    String response = respDoc["cmd"].as<String>();
                    Serial.print("Received valid response command: "); Serial.println(response);
                    lastCommTime = millis(); // Updates variable defined in this file
                    return true; 
                } else {
                    Serial.print("JSON response parsing failed or 'cmd' key missing. Error: "); Serial.println(error.c_str());
                }
            }
        }
        delay(10); 
    }
    Serial.println("Command response timeout!");
    return false;
}

/**
 * @brief Sends a command to the specified HardwareSerial port with a specified number of retries.
 * Uses the sendCommand function and retries if it returns false.
 *
 * @param serialPort The HardwareSerial port to send the command to.
 * @param cmd The command string to send.
 * @param data The data string associated with the command.
 * @param maxRetries The maximum number of times to retry sending the command.
 * @return true If the command was successfully sent and acknowledged within the retry limit.
 * @return false If the command failed even after all retries.
 */
bool sendCommandWithRetry(Stream& serialPort, const char* cmd, const String& data, int maxRetries) {
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        if (sendCommand(serialPort, cmd, data, CMD_TIMEOUT)) return true;
        Serial.print("Command failed. Retry attempt "); Serial.print(attempt); Serial.print(" of "); Serial.println(maxRetries);
        delay(500); 
    }
    Serial.print("Failed to send command '"); Serial.print(cmd); Serial.print("' after "); Serial.print(maxRetries); Serial.println(" attempts.");
    return false;
}

/**
 * @brief Checks if the device on the specified HardwareSerial port accepts operation in "No Internet" mode.
 * Sends the CMD_NONET command and expects an acknowledgment.
 *
 * @param serialPort The HardwareSerial port to query.
 * @return true If the device acknowledges the CMD_NONET command.
 * @return false If the command fails or times out.
 */
bool checkNoInternetMode(Stream& serialPort) {
    Serial.println("Querying device about No Internet mode acceptance...");
    if (sendCommand(serialPort, CMD_NONET, "", CMD_TIMEOUT)) {
        Serial.println("Device accepted No Internet mode."); return true;
    } else {
        Serial.println("Device did not accept or respond to No Internet mode query."); return false;
    }
}

/**
 * @brief Verifies communication with the device on the specified HardwareSerial port by sending an INFO command.
 * Attempts to establish communication within a specified timeout, blinking an LED
 * during the process. Retries sending the command multiple times.
 *
 * @param serialPort The HardwareSerial port to verify communication with.
 * @return true If communication is successfully established (device responds).
 * @return false If communication cannot be established within the timeout.
 */
bool verifyArduinoCommunication(Stream& serialPort) { 
    Serial.println("Attempting to establish communication with device...");
    unsigned long startTime = millis();
    const unsigned long verificationTimeout = 30000; 
    const int maxAttempts = 5; 
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        Serial.print("Verification attempt "); Serial.println(attempt);
        for (int i=0; i<5; ++i) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
        digitalWrite(LED_PIN, LOW); 
        if (sendCommand(serialPort, CMD_INFO, "", CMD_TIMEOUT)) {
            Serial.println("Communication with device established successfully!");
            lastCommTime = millis(); // Updates variable defined in this file
            digitalWrite(LED_PIN, HIGH); 
            return true;
        }
        if (millis() - startTime > verificationTimeout) break; 
        Serial.println("Verification attempt failed. Waiting before next attempt...");
        delay(2000); 
    }
    Serial.println("Failed to establish communication with device after multiple attempts!");
    digitalWrite(LED_PIN, LOW); 
    return false;
}

/**
 * @brief Sends the CMD_DATA command to the device on the specified HardwareSerial port without waiting for a response.
 * This requests the device to send its latest sensor readings.
 *
 * @param serialPort The HardwareSerial port to send the command to.
 */
void sendSensorDataCMD(Stream& serialPort) {
    Serial.println("Requesting sensor data from device...");
    sendCommandWithoutResponse(serialPort, CMD_DATA, "");
} 