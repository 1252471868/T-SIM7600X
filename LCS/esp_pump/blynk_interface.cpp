#include "global.h"
#include "blynk_interface.h"
#include "pump_control.h"
#include "system_utils.h"
#include "bluetooth_comm.h"
#include <BlynkSimpleTinyGSM.h>

// Blynk authentication and connection
char auth[] = "YOUR_BLYNK_AUTH_TOKEN"; // Replace with your actual auth token
char apn[]  = ""; // APN for your carrier
char user[] = ""; // Username (if required)
char pass[] = ""; // Password (if required)
BlynkTimer timer;

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
    // Check timed operations every second
    timer.setInterval(1000, checkTimedOperations);
    
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
            statusText = pumpOp.enabled ? (pumpOp.isInflating ? "MANUAL INFLATE" : "MANUAL DEFLATE") : "MANUAL READY";
            break;
        case PUMP_VOC_AUTO:
            statusText = "VOC AUTO MODE";
            break;
        case PUMP_TIMED:
            statusText = "TIMED MODE";
            break;
        case PUMP_SAMPLING:
            statusText = "SAMPLING";
            break;
    }
    
    Blynk.virtualWrite(VPIN_PUMP_STATUS, statusText);
    Blynk.virtualWrite(VPIN_PUMP_MODE, (int)pumpOp.mode);
    Blynk.virtualWrite(VPIN_CURRENT_VOC, vocMonitor.currentLevel);
    Blynk.virtualWrite(VPIN_ARDUINO_STATUS, arduinoConnected ? 1 : 0);
    Blynk.virtualWrite(VPIN_MAIN_ESP32_STATUS, mainESP32Connected ? 1 : 0);
    
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
    if (!vocMonitor.autoEnabled) return;
    
    // Request fresh VOC data from main ESP32
    requestVOCData();
    
    // Get the latest VOC reading from main ESP32
    vocMonitor.currentLevel = getLatestVOCReading();
    
    // Check if threshold is exceeded
    if (vocMonitor.currentLevel > vocMonitor.threshold && !pumpOp.enabled) {
        Serial.print("VOC threshold exceeded: ");
        Serial.print(vocMonitor.currentLevel);
        Serial.print(" > ");
        Serial.println(vocMonitor.threshold);
        
        // Trigger sampling mode
        startSamplingMode();
        
        if (blynkConnected) {
            Blynk.logEvent("voc_threshold_exceeded", String("VOC: ") + String(vocMonitor.currentLevel));
        }
    }
    
    vocMonitor.lastCheck = millis();
}

/**
 * @brief Checks for scheduled timed operations
 */
void checkTimedOperations() {
    if (!timedOp.enabled || !timedOp.scheduled) return;
    
    if (millis() >= timedOp.startTime) {
        Serial.println("Starting scheduled timed operation");
        startSamplingMode();
        timedOp.scheduled = false; // Reset schedule flag
        
        if (blynkConnected) {
            Blynk.logEvent("timed_sampling_started", "Scheduled sampling started");
        }
    }
}

// --- Blynk Event Handlers ---

// Virtual Pin V0 - Pump Enable/Disable
BLYNK_WRITE(VPIN_PUMP_ENABLE) {
    bool enable = param.asInt() == 1;
    Serial.print("Pump enable command: ");
    Serial.println(enable ? "ON" : "OFF");
    
    if (enable) {
        pumpOp.mode = PUMP_MANUAL;
        startPump(pumpOp.isInflating, pumpOp.flowRate);
    } else {
        stopPump();
    }
    
    Blynk.logEvent(enable ? "pump_started" : "pump_stopped", "Manual pump control");
}

// Virtual Pin V1 - Flow Rate Control
BLYNK_WRITE(VPIN_PUMP_FLOW_RATE) {
    int flowRate = param.asInt();
    if (flowRate < 0) flowRate = 0;
    if (flowRate > 255) flowRate = 255;
    
    pumpOp.flowRate = flowRate;
    Serial.print("Flow rate set to: ");
    Serial.println(flowRate);
    
    // If pump is currently running, update the flow rate
    if (pumpOp.enabled && pumpOp.mode == PUMP_MANUAL) {
        updatePumpFlowRate(flowRate);
    }
}

// Virtual Pin V2 - VOC Auto-trigger Enable
BLYNK_WRITE(VPIN_VOC_AUTO_ENABLE) {
    vocMonitor.autoEnabled = param.asInt() == 1;
    Serial.print("VOC auto-trigger: ");
    Serial.println(vocMonitor.autoEnabled ? "ENABLED" : "DISABLED");
    
    if (vocMonitor.autoEnabled) {
        pumpOp.mode = PUMP_VOC_AUTO;
        Blynk.logEvent("voc_auto_enabled", "VOC auto-trigger activated");
    } else if (pumpOp.mode == PUMP_VOC_AUTO) {
        pumpOp.mode = PUMP_OFF;
        stopPump();
    }
}

// Virtual Pin V3 - VOC Concentration Threshold
BLYNK_WRITE(VPIN_VOC_THRESHOLD) {
    float threshold = param.asFloat();
    if (threshold < 0) threshold = 0;
    if (threshold > 1000) threshold = 1000;
    
    vocMonitor.threshold = threshold;
    Serial.print("VOC threshold set to: ");
    Serial.println(threshold);
    
    Blynk.logEvent("threshold_updated", String("New threshold: ") + String(threshold));
}

// Virtual Pin V4 - Timed Start Enable
BLYNK_WRITE(VPIN_TIMED_START_ENABLE) {
    timedOp.enabled = param.asInt() == 1;
    Serial.print("Timed start: ");
    Serial.println(timedOp.enabled ? "ENABLED" : "DISABLED");
    
    if (timedOp.enabled) {
        pumpOp.mode = PUMP_TIMED;
        Blynk.logEvent("timed_mode_enabled", "Timed operation mode activated");
    } else if (pumpOp.mode == PUMP_TIMED) {
        pumpOp.mode = PUMP_OFF;
        timedOp.scheduled = false;
    }
}

// Virtual Pin V5 - Timed Start Time (minutes from now)
BLYNK_WRITE(VPIN_TIMED_START_TIME) {
    int minutesFromNow = param.asInt();
    if (minutesFromNow < 0) minutesFromNow = 0;
    if (minutesFromNow > 1440) minutesFromNow = 1440; // Max 24 hours
    
    timedOp.startTime = millis() + (minutesFromNow * 60000UL);
    timedOp.scheduled = (minutesFromNow > 0);
    
    Serial.print("Timed start scheduled for ");
    Serial.print(minutesFromNow);
    Serial.println(" minutes from now");
    
    if (timedOp.scheduled) {
        Blynk.logEvent("timed_scheduled", String("Scheduled in ") + String(minutesFromNow) + " minutes");
    }
}

// Virtual Pin V6 - Sampling Duration (seconds)
BLYNK_WRITE(VPIN_SAMPLING_TIME) {
    int seconds = param.asInt();
    if (seconds < 1) seconds = 1;
    if (seconds > 3600) seconds = 3600; // Max 1 hour
    
    timedOp.samplingDuration = seconds * 1000UL;
    Serial.print("Sampling duration set to: ");
    Serial.print(seconds);
    Serial.println(" seconds");
}

// Virtual Pin V12 - System Reset
BLYNK_WRITE(VPIN_SYSTEM_RESET) {
    if (param.asInt() == 1) {
        Serial.println("System reset requested from Blynk");
        Blynk.logEvent("system_reset", "System reset initiated from app");
        delay(1000);
        resetSystem();
    }
}

// Virtual Pin V13 - Emergency Stop
BLYNK_WRITE(VPIN_EMERGENCY_STOP) {
    if (param.asInt() == 1) {
        Serial.println("EMERGENCY STOP activated from Blynk");
        emergencyStop();
        Blynk.logEvent("emergency_stop", "Emergency stop activated");
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
    Blynk.syncVirtual(VPIN_VOC_AUTO_ENABLE);
    Blynk.syncVirtual(VPIN_VOC_THRESHOLD);
    Blynk.syncVirtual(VPIN_TIMED_START_ENABLE);
    Blynk.syncVirtual(VPIN_SAMPLING_TIME);
} 