#include "global.h"
#include "pump_control.h"
#include "communication.h"

/**
 * @brief Verifies Arduino connection by sending INFO command
 * @return true if Arduino responds within timeout
 */
bool verifyArduinoConnection() {
    Serial.println("Verifying Arduino connection...");
    return verifyArduinoCommunication(ArduinoSerial);
}

/**
 * @brief Starts pump operation
 * @param inflate true for inflation, false for deflation
 * @param flowRate PWM duty cycle (0-255)
 */
void startPump(bool inflate, int flowRate) {
    Serial.print("Starting pump - Mode: ");
    Serial.print(inflate ? "INFLATE" : "DEFLATE");
    Serial.print(", Flow Rate: ");
    Serial.println(flowRate);
    
    // Validate flow rate
    if (flowRate < 0) flowRate = 0;
    if (flowRate > 255) flowRate = 255;
    
    // Send command to Arduino using shared communication functions
    const char* cmd = inflate ? CMD_INFLATE : CMD_DEFLATE;
    sendCommandWithoutResponse(ArduinoSerial, cmd, String(flowRate));
    
    pumpOp.enabled = true;
    pumpOp.isInflating = inflate;
    pumpOp.flowRate = flowRate;
    pumpOp.startTime = millis();
    lastArduinoComm = millis();
    
    Serial.println("Pump started successfully");
}

/**
 * @brief Stops pump operation
 */
void stopPump() {
    Serial.println("Stopping pump");
    
    sendCommandWithoutResponse(ArduinoSerial, CMD_STOP, "");
    
    pumpOp.enabled = false;
    pumpOp.startTime = 0;
    pumpOp.duration = 0;
    lastArduinoComm = millis();
    
    Serial.println("Pump stopped");
}

/**
 * @brief Updates pump flow rate while running
 * @param flowRate New PWM duty cycle (0-255)
 */
void updatePumpFlowRate(int flowRate) {
    if (!pumpOp.enabled) {
        Serial.println("Cannot update flow rate - pump not running");
        return;
    }
    
    Serial.print("Updating flow rate to: ");
    Serial.println(flowRate);
    
    // Validate flow rate
    if (flowRate < 0) flowRate = 0;
    if (flowRate > 255) flowRate = 255;
    
    // Send new command with updated flow rate
    const char* cmd = pumpOp.isInflating ? CMD_INFLATE : CMD_DEFLATE;
    sendCommandWithoutResponse(ArduinoSerial, cmd, String(flowRate));
    
    pumpOp.flowRate = flowRate;
    lastArduinoComm = millis();
}

/**
 * @brief Starts sampling mode (auto-triggered by VOC or timed)
 */
void startSamplingMode() {
    Serial.println("Starting sampling mode");
    
    pumpOp.mode = PUMP_SAMPLING;
    pumpOp.duration = timedOp.samplingDuration;
    
    // Start with inflation at default flow rate
    startPump(true, pumpOp.flowRate);
    
    Serial.print("Sampling will run for ");
    Serial.print(timedOp.samplingDuration / 1000);
    Serial.println(" seconds");
}

/**
 * @brief Emergency stop - immediately stops all operations
 */
void emergencyStop() {
    Serial.println("EMERGENCY STOP ACTIVATED");
    
    // Immediately stop pump
    stopPump();
    
    // Reset all modes and timers
    pumpOp.mode = PUMP_OFF;
    pumpOp.enabled = false;
    vocMonitor.autoEnabled = false;
    timedOp.enabled = false;
    timedOp.scheduled = false;
    
    Serial.println("All pump operations halted");
}

/**
 * @brief Processes ongoing pump operations and timers
 * Should be called regularly in main loop
 */
void processPumpOperations() {
    // Check Arduino connection status
    if (millis() - lastArduinoComm > CMD_TIMEOUT) {
        if (arduinoConnected) {
            Serial.println("Arduino communication timeout");
            arduinoConnected = false;
        }
    } else {
        arduinoConnected = true;
    }
    
    // Check main ESP32 connection status
    if (millis() - lastMainESP32Comm > CMD_TIMEOUT) {
        if (mainESP32Connected) {
            Serial.println("Main ESP32 communication timeout");
            mainESP32Connected = false;
        }
    }
    
    // Check if sampling mode duration has expired
    if (pumpOp.enabled && pumpOp.mode == PUMP_SAMPLING && pumpOp.duration > 0) {
        if (millis() - pumpOp.startTime >= pumpOp.duration) {
            Serial.println("Sampling duration completed");
            stopPump();
            pumpOp.mode = PUMP_OFF;
        }
    }
    
    // Safety check - if pump has been running too long in manual mode (failsafe)
    if (pumpOp.enabled && pumpOp.mode == PUMP_MANUAL) {
        const unsigned long MAX_MANUAL_RUNTIME = 300000; // 5 minutes max
        if (millis() - pumpOp.startTime >= MAX_MANUAL_RUNTIME) {
            Serial.println("Manual pump runtime limit reached - auto-stopping");
            stopPump();
            pumpOp.mode = PUMP_OFF;
        }
    }
} 