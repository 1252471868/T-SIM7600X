#include "global.h"
#include "blynk_interface.h"
#include "pump_control.h"
#include "system_utils.h"
#include <BlynkSimpleTinyGSM.h>

// Blynk authentication and connection
char auth[] = BLYNK_AUTH_TOKEN; // Replace with your actual auth token
char apn[]  = ""; // APN for your carrier
char user[] = ""; // Username (if required)
char pass[] = ""; // Password (if required)
BlynkTimer timer;

/**
 * @brief Updates Blynk pump enable status (important for UI synchronization)
 */
void updateBlynkPumpStatus(bool enabled) {
    if (blynkConnected) {
        Blynk.virtualWrite(VPIN_PUMP_ENABLE, enabled ? 1 : 0);
        Serial.print("Updated Blynk pump enable status to: ");
        Serial.println(enabled ? "ON" : "OFF");
    }
}

/**
 * @brief Initializes the Blynk connection.
 * Attempts to connect to GPRS and then the Blynk server.
 * @return true if successfully connected to Blynk, false otherwise.
 */
bool initializeBlynk() {
    bool blynkInitialized = false;
    Serial.println("Initializing Blynk connection...");
    Blynk.config(modem, auth, BLYNK_DEFAULT_DOMAIN, BLYNK_DEFAULT_PORT);
    
    Serial.println("Connecting to GPRS network...");
    if(Blynk.connectNetwork(apn, user, pass)) {
        Serial.println("GPRS network connected. Connecting to Blynk...");
        if (Blynk.connect(BLYNK_CONNECT_TIMEOUT)) {
            blynkConnected = true;
            internetAvailable = true;
            Serial.println("Successfully connected to Blynk!");
            blynkInitialized = true;
        } else {
            Serial.println("Failed to connect to Blynk server.");
            blynkConnected = false;
            internetAvailable = false;
            blynkInitialized = false;
        }
    } else {
        Serial.println("Failed to connect to GPRS network.");
        blynkConnected = false;
        internetAvailable = false;
        blynkInitialized = false;
    }

    // Fallback logic if Blynk connection failed
    if (!blynkInitialized || !internetAvailable) {
        Serial.println("Blynk/Internet initialization failed. Operating in offline mode.");
        internetAvailable = false;
    } else {
        internetAvailable = true;
    }
    
    // Setup timers for periodic tasks
    Serial.println("Setting up timers...");
    // Update status every 5 seconds
    timer.setInterval(BLYNK_SEND_INTERVAL, updateBlynkStatus);
    // Check VOC levels every 10 seconds if auto mode is enabled
    timer.setInterval(10000, checkVOCLevels);
    // Check timed operations every 30 seconds
    timer.setInterval(30000, checkTimedOperations);
    
    return blynkInitialized;
}

/**
 * @brief Runs the main Blynk task loop.
 * Should be called repeatedly in the main loop().
 */
void runBlynk() {
    if (blynkConnected) {
        Blynk.run();
    }
    timer.run(); // Run BlynkTimer tasks
}

/**
 * @brief Updates pump status and data to Blynk dashboard
 */
void updateBlynkStatus() {
    if (!blynkConnected) return;
    
    // Update pump status display
    String statusText = "";
    switch (pumpOp.mode) {
        case PUMP_OFF:
            statusText = "OFF";
            break;
        case PUMP_MANUAL:
            if (pumpOp.enabled) {
                statusText = pumpOp.isInflating ? "MANUAL INFLATE" : "MANUAL DEFLATE";
                if (pumpOp.duration > 0) {
                    unsigned long remaining = (millis() - pumpOp.startTime >= pumpOp.duration) ? 0 : 
                                            (pumpOp.duration - (millis() - pumpOp.startTime));
                    statusText += " (" + String(remaining / 1000) + "s left)";
                } else {
                    statusText += " (Continuous)";
                }
            } else {
                statusText = "MANUAL READY";
            }
            break;
        case PUMP_VOC_AUTO:
            statusText = pumpOp.enabled ? "VOC AUTO SAMPLING" : "VOC AUTO MONITORING";
            break;
        case PUMP_TIMED:
            statusText = pumpOp.enabled ? "TIMED SAMPLING" : 
                        (timedOp.isActive ? "TIMED ACTIVE" : "TIMED WAITING");
            break;
    }
    
    Blynk.virtualWrite(VPIN_PUMP_STATUS, statusText);
    Blynk.virtualWrite(VPIN_PUMP_MODE, (int)pumpOp.mode);
    Blynk.virtualWrite(VPIN_CURRENT_VOC, vocMonitor.currentLevel);
    
    // Check if VOC data is recent (within last 90 seconds) - indicates HTTP API communication
    unsigned long timeSinceLastVOC = millis() - vocMonitor.lastCheck;
    bool vocDataFresh = (timeSinceLastVOC < 90000);
    Blynk.virtualWrite(VPIN_MAIN_ESP32_STATUS, vocDataFresh ? 1 : 0);
    
    Serial.println("Status updated to Blynk");
}

/**
 * @brief Sends pump data to Blynk (called periodically)
 */
void sendPumpDataToBlynk() {
    updateBlynkStatus();
}

/**
 * @brief Checks VOC levels for auto-triggering
 */
void checkVOCLevels() {
    if (pumpOp.mode != PUMP_VOC_AUTO) return;
    
    // VOC data is received via HTTP API from main ESP32 to virtual pin V16
    unsigned long timeSinceLastVOC = millis() - vocMonitor.lastCheck;
    if (timeSinceLastVOC > 90000) {
        Serial.println("Warning: No recent VOC data received from main ESP32 via HTTP API");
        return;
    }
    
    Serial.print("VOC auto-trigger monitoring. Current level: ");
    Serial.print(vocMonitor.currentLevel);
    Serial.print(", Threshold: ");
    Serial.print(vocMonitor.threshold);
    Serial.print(", Data age: ");
    Serial.print(timeSinceLastVOC / 1000);
    Serial.println(" seconds");
}

/**
 * @brief Checks for scheduled timed operations
 */
void checkTimedOperations() {
    if (pumpOp.mode != PUMP_TIMED) return;
    
    // The actual timed operation logic is handled in processPumpOperations()
    // This function just provides periodic status updates
    if (timedOp.isActive) {
        Serial.print("Timed operation active. Window: ");
        Serial.printf("%02d:%02d:%02d - %02d:%02d:%02d", 
                     timedOp.startHour, timedOp.startMinute, timedOp.startSecond,
                     timedOp.endHour, timedOp.endMinute, timedOp.endSecond);
        Serial.print(", Pump enabled: ");
        Serial.println(pumpOp.enabled ? "YES" : "NO");
    }
}

// --- Blynk Event Handlers ---

// Virtual Pin V0 - Pump Enable/Disable
BLYNK_WRITE(VPIN_PUMP_ENABLE) {
    bool enable = param.asInt() == 1;
    Serial.print("Pump enable command: ");
    Serial.println(enable ? "ON" : "OFF");
    
    if (enable) {
        // Start pump based on current mode
        if (pumpOp.mode == PUMP_MANUAL) {
            startPump(pumpOp.isInflating, pumpOp.flowRate);
        } else {
            Serial.println("Cannot manually enable pump - not in manual mode");
            updateBlynkPumpStatus(false); // Reset button
        }
    } else {
        stopPump();
    }
    
    Blynk.logEvent(enable ? "pump_started" : "pump_stopped", "Manual pump control");
}

// Virtual Pin V1 - Flow Rate Control
BLYNK_WRITE(VPIN_PUMP_FLOW_RATE) {
    double flowRateLPM = param.asDouble();
    
    // Validate input range (0-2.5 LPM)
    if (flowRateLPM < 0.0) flowRateLPM = 0.0;
    if (flowRateLPM > 2.5) flowRateLPM = 2.5;
    
    // Convert from 0-2.5 LPM to 0-255 duty cycle
    int flowRateDutyCycle = (int)((flowRateLPM / 2.5) * 255.0);
    
    pumpOp.flowRate = flowRateDutyCycle;
    Serial.print("Flow rate received: ");
    Serial.print(flowRateLPM);
    Serial.print(" LPM, converted to duty cycle: ");
    Serial.println(flowRateDutyCycle);
    
    // If pump is currently running, update the flow rate
    if (pumpOp.enabled) {
        updatePumpFlowRate(flowRateDutyCycle);
    }
}

// Virtual Pin V2 - Operation Mode Selector
BLYNK_WRITE(VPIN_PUMP_MODE) {
    int mode = param.asInt();
    Serial.print("Mode change requested - Blynk value: ");
    Serial.print(mode);
    Serial.print(" -> ");
    
    // Stop current operation before mode change
    if (pumpOp.enabled) {
        stopPump();
        updateBlynkPumpStatus(false);
    }
    
    switch (mode) {
        case 0: // Manual mode (PUMP_MANUAL = 0)
            pumpOp.mode = PUMP_MANUAL;
            Serial.println("PUMP_MANUAL");
            break;
        case 1: // VOC Auto mode (PUMP_VOC_AUTO = 1)
            pumpOp.mode = PUMP_VOC_AUTO;
            Serial.println("PUMP_VOC_AUTO");
            break;
        case 2: // Timed mode (PUMP_TIMED = 2)
            pumpOp.mode = PUMP_TIMED;
            Serial.println("PUMP_TIMED");
            break;
        default: // Invalid mode value - set to OFF
            pumpOp.mode = PUMP_OFF;
            Serial.print("INVALID MODE (");
            Serial.print(mode);
            Serial.println(") -> PUMP_OFF");
            break;
    }
    
    Blynk.logEvent("mode_changed", String("Mode: ") + String(mode) + " -> " + String((int)pumpOp.mode));
}

// Virtual Pin V3 - Sampling Duration (applies to all modes)
BLYNK_WRITE(VPIN_SAMPLING_DURATION) {
    int durationSeconds = param.asInt();
    if (durationSeconds < 0) durationSeconds = 0;
    if (durationSeconds > 3600) durationSeconds = 3600; // Max 1 hour
    
    pumpOp.duration = durationSeconds * 1000UL; // Convert to milliseconds
    
    Serial.print("Sampling duration set to: ");
    if (durationSeconds == 0) {
        Serial.println("Continuous (manual mode only)");
    } else {
        Serial.print(durationSeconds);
        Serial.println(" seconds (applies to all modes)");
    }
    
    Serial.println("Note: 0=continuous operation (manual mode only), >0=timed operation (all modes)");
}

// Virtual Pin V5 - VOC Concentration Threshold
BLYNK_WRITE(VPIN_VOC_THRESHOLD) {
    float threshold = param.asFloat();
    if (threshold < 0) threshold = 0;
    if (threshold > 1000) threshold = 1000;
    
    vocMonitor.threshold = threshold;
    Serial.print("VOC threshold set to: ");
    Serial.println(threshold);
    
    Blynk.logEvent("threshold_updated", String("New threshold: ") + String(threshold));
}

// Virtual Pin V8 - Timed Operation Time Range (Time Input Widget)
BLYNK_WRITE(VPIN_TIMED_TIME_RANGE) {
    // Blynk Time Input widget sends multiple parameters:
    // param[0] = start time in seconds since midnight
    // param[1] = stop time in seconds since midnight  
    // param[2] = timezone (optional)
    // param[3] = day of week (optional)
    // param[4] = UTC offset (optional)
    
    Serial.println("DEBUG: Received Time Input widget data:");
    
    // Get start time in seconds since midnight
    uint32_t startTimeSeconds = param[0].asInt();
    Serial.print("DEBUG: Start time in seconds: ");
    Serial.println(startTimeSeconds);
    
    // Get stop time in seconds since midnight
    uint32_t stopTimeSeconds = param[1].asInt();
    Serial.print("DEBUG: Stop time in seconds: ");
    Serial.println(stopTimeSeconds);
    
    // Convert start time from seconds to hours:minutes:seconds
    int startHour = startTimeSeconds / 3600;
    int startMinute = (startTimeSeconds % 3600) / 60;
    int startSecond = startTimeSeconds % 60;
    
    // Convert stop time from seconds to hours:minutes:seconds
    int stopHour = stopTimeSeconds / 3600;
    int stopMinute = (stopTimeSeconds % 3600) / 60;
    int stopSecondValue = stopTimeSeconds % 60;
    
    Serial.printf("DEBUG: Converted start time: %02d:%02d:%02d\n", startHour, startMinute, startSecond);
    Serial.printf("DEBUG: Converted stop time: %02d:%02d:%02d\n", stopHour, stopMinute, stopSecondValue);
    
    // Validate time ranges
    if (startHour < 0 || startHour > 23 || startMinute < 0 || startMinute > 59 || startSecond < 0 || startSecond > 59) {
        Serial.printf("DEBUG: Invalid start time - H:%d (0-23), M:%d (0-59), S:%d (0-59)\n", startHour, startMinute, startSecond);
        return;
    }
    
    if (stopHour < 0 || stopHour > 23 || stopMinute < 0 || stopMinute > 59 || stopSecondValue < 0 || stopSecondValue > 59) {
        Serial.printf("DEBUG: Invalid stop time - H:%d (0-23), M:%d (0-59), S:%d (0-59)\n", stopHour, stopMinute, stopSecondValue);
        return;
    }
    
    // Update global timed operation settings
    timedOp.startHour = startHour;
    timedOp.startMinute = startMinute;
    timedOp.startSecond = startSecond;
    timedOp.endHour = stopHour;
    timedOp.endMinute = stopMinute;
    timedOp.endSecond = stopSecondValue;
    
    Serial.print("Timed operation time range set to: ");
    Serial.printf("%02d:%02d:%02d - %02d:%02d:%02d\n", 
                 startHour, startMinute, startSecond,
                 stopHour, stopMinute, stopSecondValue);
    
    // Optional: Log timezone and day information if available
    if (param.getLength() > 2) {
        String timezone = param[2].asStr();
        Serial.print("DEBUG: Timezone: ");
        Serial.println(timezone);
    }
    
    if (param.getLength() > 3) {
        int dayOfWeek = param[3].asInt();
        Serial.print("DEBUG: Day of week: ");
        Serial.println(dayOfWeek);
    }
    
    Blynk.logEvent("timed_range_updated", String("Range: ") + 
                   String(startHour) + ":" + String(startMinute) + ":" + String(startSecond) + "-" +
                   String(stopHour) + ":" + String(stopMinute) + ":" + String(stopSecondValue));
}

// Virtual Pin V14 - System Reset
BLYNK_WRITE(VPIN_SYSTEM_RESET) {
    if (param.asInt() == 1) {
        Serial.println("System reset requested from Blynk");
        Blynk.logEvent("system_reset", "System reset initiated from app");
        delay(1000);
        resetSystem();
    }
}

// Virtual Pin V15 - Emergency Stop
BLYNK_WRITE(VPIN_EMERGENCY_STOP) {
    if (param.asInt() == 1) {
        Serial.println("EMERGENCY STOP activated from Blynk");
        emergencyStop();
        Blynk.logEvent("emergency_stop", "Emergency stop activated");
    }
}

// Virtual Pin V16 - VOC Data from Main ESP32 (HTTP API)
BLYNK_WRITE(V16) {
    float vocValue = param.asFloat();
    
    // Update VOC monitoring with received data
    vocMonitor.currentLevel = vocValue;
    vocMonitor.lastCheck = millis();
    
    Serial.print("Received VOC data from main ESP32 via HTTP API: ");
    Serial.println(vocValue);
    
    // Update the display virtual pin
    if (blynkConnected) {
        Blynk.virtualWrite(VPIN_CURRENT_VOC, vocValue);
    }
    
    // Check if auto-trigger threshold is exceeded (only in VOC AUTO mode)
    if (pumpOp.mode == PUMP_VOC_AUTO && !pumpOp.enabled && vocValue > vocMonitor.threshold) {
        Serial.print("VOC threshold exceeded via HTTP API: ");
        Serial.print(vocValue);
        Serial.print(" > ");
        Serial.println(vocMonitor.threshold);
        
        // Trigger VOC sampling
        startVOCTriggeredSampling();
        
        if (blynkConnected) {
            Blynk.logEvent("voc_threshold_exceeded", String("VOC via HTTP API: ") + String(vocValue));
        }
    }
}

// Blynk connection events
BLYNK_CONNECTED() {
    Serial.println("Successfully connected to Blynk server.");
    blynkConnected = true;
    internetAvailable = true;
    
    // Sync virtual pins on reconnection
    Blynk.syncVirtual(VPIN_PUMP_ENABLE);
    Blynk.syncVirtual(VPIN_PUMP_FLOW_RATE);
    Blynk.syncVirtual(VPIN_PUMP_MODE);
    Blynk.syncVirtual(VPIN_SAMPLING_DURATION);
    Blynk.syncVirtual(VPIN_VOC_THRESHOLD);
    Blynk.syncVirtual(VPIN_TIMED_TIME_RANGE);
} 