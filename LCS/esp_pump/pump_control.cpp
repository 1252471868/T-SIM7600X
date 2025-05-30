#include "global.h"
#include "pump_control.h"
#include "blynk_interface.h"

/**
 * @brief Initializes pump control pins and hardware for L298N motor drivers
 */
void initializePumpControl() {
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    Serial.println("Initializing L298N motor drivers - DUAL PUMP WITH DEFLATION...");
    
    // Initialize Pump 1 pins (L298N Motor A)
    pinMode(PUMP1_ENA_PIN, OUTPUT);  // Enable pin for PWM speed control
    pinMode(PUMP1_IN1_PIN, OUTPUT);  // Single direction control
    
    // Initialize Pump 2 pins (L298N Motor B)
    pinMode(PUMP2_ENA_PIN, OUTPUT);  // Enable pin for PWM speed control
    pinMode(PUMP2_IN1_PIN, OUTPUT);  // Single direction control
    
    // Initialize valve pins (L298N Motor C/D)
    pinMode(VALVE_ENA_PIN, OUTPUT);  // Enable pin for 5V power
    pinMode(VALVE_IN1_PIN, OUTPUT);  // Digital ON/OFF control
    
    Serial.println("Dual pump system pins initialized:");
    Serial.println("  Pump 1 - ENA: 22, IN1: 21");
    Serial.println("  Pump 2 - ENA: 23, IN1: 19");
    Serial.println("  Valve - ENA: 18, IN1: 5 (5V via L298N)");
    Serial.println("  Total pins used: 6 (saved 1 pin)");
    Serial.println("  Capabilities: Inflation + Deflation");
    
#elif PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    Serial.println("Initializing L298N motor driver - SINGLE PUMP INFLATION ONLY...");
    
    // Initialize Pump 1 pins only (L298N Motor A)
    pinMode(PUMP1_ENA_PIN, OUTPUT);  // Enable pin for PWM speed control
    pinMode(PUMP1_IN1_PIN, OUTPUT);  // Single direction control
    
    // Initialize valve pins (L298N Motor B)
    pinMode(VALVE_ENA_PIN, OUTPUT);  // Enable pin for 5V power
    pinMode(VALVE_IN1_PIN, OUTPUT);  // Digital ON/OFF control
    
    Serial.println("Single pump system pins initialized:");
    Serial.println("  Pump 1 - ENA: 22, IN1: 21");
    Serial.println("  Valve - ENA: 18, IN1: 5 (5V via L298N)");
    Serial.println("  Total pins used: 4");
    Serial.println("  Capabilities: Inflation only (no deflation)");
    Serial.print("  Valve-Pump start delay: ");
    Serial.print(VALVE_PUMP_START_DELAY);
    Serial.println(" ms");
    Serial.print("  Pump-Valve stop delay: ");
    Serial.print(PUMP_VALVE_STOP_DELAY);
    Serial.println(" ms");
#endif
    
    // Ensure everything is off initially
    stopPump();
}

/**
 * @brief Verifies pump hardware (always returns true for direct control)
 * @return true always (no external communication needed)
 */
bool verifyPumpHardware() {
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    Serial.println("Verifying dual pump system with deflation capability...");
#elif PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    Serial.println("Verifying single pump inflation-only system...");
#endif
    
    // Test all pins by setting them to known states
    stopAllPumpsAndValve();
    
    Serial.println("L298N motor driver hardware verification complete");
    return true;
}

/**
 * @brief Set pump control using single direction L298N control
 * @param pumpNumber 1 or 2 for pump selection (dual pump), only 1 for single pump
 * @param enabled true to enable pump, false to disable
 * @param dutyCycle PWM duty cycle (0-255) for speed control
 */
void setPumpControl(int pumpNumber, bool enabled, int dutyCycle) {
    // Validate duty cycle
    if (dutyCycle < 0) dutyCycle = 0;
    if (dutyCycle > 255) dutyCycle = 255;
    
    if (pumpNumber == 1) {
        // Control Pump 1 (Motor A)
        if (enabled && dutyCycle > 0) {
            analogWrite(PUMP1_ENA_PIN, dutyCycle);  // Set speed via PWM
            digitalWrite(PUMP1_IN1_PIN, HIGH);      // Enable motor
        } else {
            analogWrite(PUMP1_ENA_PIN, 0);          // Stop PWM
            digitalWrite(PUMP1_IN1_PIN, LOW);       // Disable motor
        }
    }
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    else if (pumpNumber == 2) {
        // Control Pump 2 (Motor B) - only available in dual pump system
        if (enabled && dutyCycle > 0) {
            analogWrite(PUMP2_ENA_PIN, dutyCycle);  // Set speed via PWM
            digitalWrite(PUMP2_IN1_PIN, HIGH);      // Enable motor
        } else {
            analogWrite(PUMP2_ENA_PIN, 0);          // Stop PWM
            digitalWrite(PUMP2_IN1_PIN, LOW);       // Disable motor
        }
    }
#endif
}

/**
 * @brief Control valve using L298N motor driver for 5V power
 * @param enabled true to open valve, false to close valve
 */
void setValveControl(bool enabled) {
    if (enabled) {
        analogWrite(VALVE_ENA_PIN, 255);         // Full duty cycle for 5V power
        digitalWrite(VALVE_IN1_PIN, HIGH);       // Enable valve
        Serial.println("Valve opened (5V via L298N)");
    } else {
        analogWrite(VALVE_ENA_PIN, 0);           // Stop PWM
        digitalWrite(VALVE_IN1_PIN, LOW);        // Disable valve
        Serial.println("Valve closed");
    }
}

/**
 * @brief Execute inflate operation with specified duty cycle
 * @param dutyCycle PWM duty cycle (0-255) for pump speed control
 */
void executeInflate(int dutyCycle) {
    Serial.print("Starting inflation with duty cycle: ");
    Serial.println(dutyCycle);
    
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    // Dual pump system: Enable both pumps for inflation
    setPumpControl(1, true, dutyCycle);   // Pump 1 enabled
    setPumpControl(2, true, dutyCycle);   // Pump 2 enabled
    
    // Open valve for inflation path
    setValveControl(true);
    
#elif PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    // Single pump system: Sequenced operation - valve first, then pump
    Serial.println("Single pump system: Opening valve first...");
    setValveControl(true);
    
    delay(VALVE_PUMP_START_DELAY);  // Wait for valve to open
    
    Serial.println("Single pump system: Starting pump after delay...");
    setPumpControl(1, true, dutyCycle);   // Pump 1 enabled only
#endif
    
    Serial.println("Inflation started successfully");
}

/**
 * @brief Execute deflate operation with specified duty cycle
 * @param dutyCycle PWM duty cycle (0-255) for pump speed control
 */
void executeDeflate(int dutyCycle) {
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    Serial.print("Starting deflation with duty cycle: ");
    Serial.println(dutyCycle);
    
    // Enable both pumps for deflation
    setPumpControl(1, true, dutyCycle);   // Pump 1 enabled
    setPumpControl(2, true, dutyCycle);   // Pump 2 enabled
    
    // Close valve for deflation path
    setValveControl(false);
    
    Serial.println("Deflation started successfully");
    
#elif PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    Serial.println("Single pump system: Deflation not supported - inflation only system");
    // In single pump inflation-only system, we don't support deflation
#endif
}

/**
 * @brief Stop all pumps and valve
 */
void stopAllPumpsAndValve() {
    Serial.println("Stopping all pumps and valve");
    
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    // Dual pump system: Stop both pumps immediately
    setPumpControl(1, false, 0);  // Stop pump 1
    setPumpControl(2, false, 0);  // Stop pump 2
    
    // Close valve
    setValveControl(false);
    
#elif PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    // Single pump system: Sequenced stop - pump first, then valve
    Serial.println("Single pump system: Stopping pump first...");
    setPumpControl(1, false, 0);  // Stop pump 1
    
    delay(PUMP_VALVE_STOP_DELAY);  // Wait for pump to stop
    
    Serial.println("Single pump system: Closing valve after delay...");
    setValveControl(false);
#endif
    
    Serial.println("All pumps and valve stopped");
}

/**
 * @brief Starts pump operation
 * @param inflate true for inflation, false for deflation
 * @param flowRate PWM duty cycle (0-255)
 */
void startPump(bool inflate, int flowRate) {
#if PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    if (!inflate) {
        Serial.println("Single pump system: Deflation not supported - switching to inflation mode");
        inflate = true;  // Force inflation mode in single pump system
    }
#endif
    
    Serial.print("Starting pump - Mode: ");
    Serial.print(inflate ? "INFLATE" : "DEFLATE");
    Serial.print(", Flow Rate: ");
    Serial.println(flowRate);
    
    // Validate flow rate
    if (flowRate < 0) flowRate = 0;
    if (flowRate > 255) flowRate = 255;
    
    // Execute the operation directly
    if (inflate) {
        executeInflate(flowRate);
    } else {
        executeDeflate(flowRate);
    }
    
    // Update pump operation status
    pumpOp.enabled = true;
    pumpOp.isInflating = inflate;
    pumpOp.flowRate = flowRate;
    pumpOp.startTime = millis();
    
    Serial.println("Pump started successfully");
}

/**
 * @brief Stops pump operation
 */
void stopPump() {
    Serial.println("Stopping pump");
    
    // Stop all pumps and valve directly
    stopAllPumpsAndValve();
    
    // Update pump operation status
    pumpOp.enabled = false;
    pumpOp.startTime = 0;
    pumpOp.duration = 0;
    
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
    
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    // Update both pumps with new speed
    setPumpControl(1, true, flowRate);
    setPumpControl(2, true, flowRate);
    
#elif PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    // Update single pump with new speed
    setPumpControl(1, true, flowRate);
#endif
    
    // Update stored flow rate
    pumpOp.flowRate = flowRate;
    
    Serial.println("Flow rate updated successfully");
}

/**
 * @brief Starts VOC-triggered sampling mode
 */
void startVOCTriggeredSampling() {
    Serial.println("Starting VOC-triggered sampling");
    
    pumpOp.mode = PUMP_VOC_AUTO;
    // Use the unified duration setting
    
    // Start with inflation at current flow rate
    startPump(true, pumpOp.flowRate);
    
    Serial.print("VOC-triggered sampling will run for ");
    if (pumpOp.duration > 0) {
        Serial.print(pumpOp.duration / 1000);
        Serial.println(" seconds");
    } else {
        Serial.println("continuous operation (until manually stopped)");
    }
}

/**
 * @brief Starts timed sampling mode
 */
void startTimedSampling() {
    Serial.println("Starting timed sampling");
    
    pumpOp.mode = PUMP_TIMED;
    timedOp.lastSampleStart = millis();
    
    // Start with inflation at current flow rate
    startPump(true, pumpOp.flowRate);
    
    Serial.print("Timed sampling will run for ");
    if (pumpOp.duration > 0) {
        Serial.print(pumpOp.duration / 1000);
        Serial.println(" seconds");
    } else {
        Serial.println("continuous operation (until manually stopped)");
    }
}

/**
 * @brief Checks if current time is within the scheduled timed operation window
 * @return true if within the scheduled time window
 */
bool isWithinTimedWindow() {
    time_t currentTime = now();
    int currentHour = hour(currentTime);
    int currentMinute = minute(currentTime);
    int currentSecond = second(currentTime);
    
    // Convert times to seconds since midnight for easier comparison
    int currentTimeSeconds = currentHour * 3600 + currentMinute * 60 + currentSecond;
    int startTimeSeconds = timedOp.startHour * 3600 + timedOp.startMinute * 60 + timedOp.startSecond;
    int endTimeSeconds = timedOp.endHour * 3600 + timedOp.endMinute * 60 + timedOp.endSecond;
    
    // Handle case where end time is next day (crosses midnight)
    if (endTimeSeconds < startTimeSeconds) {
        return (currentTimeSeconds >= startTimeSeconds || currentTimeSeconds <= endTimeSeconds);
    } else {
        return (currentTimeSeconds >= startTimeSeconds && currentTimeSeconds <= endTimeSeconds);
    }
}

/**
 * @brief Emergency stop - immediately stops all operations
 */
void emergencyStop() {
    Serial.println("EMERGENCY STOP ACTIVATED");
    
    // Immediately stop all pumps and valve
    stopPump();
    
    // Reset all modes and timers
    pumpOp.mode = PUMP_OFF;
    pumpOp.enabled = false;
    timedOp.isActive = false;
    
    // Update Blynk status to reflect emergency stop
    updateBlynkPumpStatus(false);
    
    Serial.println("All pump operations halted");
}

/**
 * @brief Processes ongoing pump operations and timers
 * Should be called regularly in main loop
 */
void processPumpOperations() {
    // Check main ESP32 connection status
    if (millis() - lastMainESP32Comm > CMD_TIMEOUT) {
        if (mainESP32Connected) {
            Serial.println("Main ESP32 communication timeout");
            mainESP32Connected = false;
        }
    } else {
        mainESP32Connected = true;
    }
    
    // Check if pump operation duration has expired (for all modes with duration)
    if (pumpOp.enabled && pumpOp.duration > 0) {
        if (millis() - pumpOp.startTime >= pumpOp.duration) {
            Serial.print("Operation duration completed for mode: ");
            switch (pumpOp.mode) {
                case PUMP_MANUAL: Serial.println("MANUAL"); break;
                case PUMP_VOC_AUTO: Serial.println("VOC AUTO"); break;
                case PUMP_TIMED: Serial.println("TIMED"); break;
                default: Serial.println("UNKNOWN"); break;
            }
            
            stopPump();
            
            // Update Blynk status to reflect auto-stop
            updateBlynkPumpStatus(false);
            
            // For VOC and timed modes, return to monitoring state
            if (pumpOp.mode == PUMP_VOC_AUTO || pumpOp.mode == PUMP_TIMED) {
                pumpOp.mode = (pumpOp.mode == PUMP_VOC_AUTO) ? PUMP_VOC_AUTO : PUMP_TIMED;
                // Mode stays active for potential future triggering
            } else {
                pumpOp.mode = PUMP_OFF;
            }
        }
    }
    
    // Safety check - if pump has been running too long in manual mode without duration (failsafe)
    if (pumpOp.enabled && pumpOp.mode == PUMP_MANUAL && pumpOp.duration == 0) {
        const unsigned long MAX_CONTINUOUS_RUNTIME = 1800000; // 30 minutes max for continuous operation
        if (millis() - pumpOp.startTime >= MAX_CONTINUOUS_RUNTIME) {
            Serial.println("Manual continuous runtime limit reached - auto-stopping for safety");
            stopPump();
            updateBlynkPumpStatus(false);
            pumpOp.mode = PUMP_OFF;
        }
    }
    
    // Process VOC auto mode
    if (pumpOp.mode == PUMP_VOC_AUTO && !pumpOp.enabled) {
        // Check if VOC threshold is exceeded and we're not currently sampling
        if (vocMonitor.currentLevel >= vocMonitor.threshold) {
            Serial.println("VOC threshold exceeded - starting auto sampling");
            startVOCTriggeredSampling();
        }
    }
    
    // Process timed mode
    if (pumpOp.mode == PUMP_TIMED) {
        bool currentlyInWindow = isWithinTimedWindow();
        
        if (currentlyInWindow && !timedOp.isActive) {
            // Entering timed window
            timedOp.isActive = true;
            Serial.println("Entered timed operation window");
        } else if (!currentlyInWindow && timedOp.isActive) {
            // Exiting timed window
            timedOp.isActive = false;
            if (pumpOp.enabled) {
                Serial.println("Exited timed operation window - stopping pump");
                stopPump();
                updateBlynkPumpStatus(false);
            }
        }
        
        // If in window and not currently sampling, start sampling
        if (timedOp.isActive && !pumpOp.enabled) {
            // Check if enough time has passed since last sample (avoid continuous operation)
            const unsigned long MIN_INTERVAL_BETWEEN_SAMPLES = 300000; // 5 minutes minimum
            if (millis() - timedOp.lastSampleStart >= MIN_INTERVAL_BETWEEN_SAMPLES) {
                Serial.println("Starting scheduled timed sampling");
                startTimedSampling();
            }
        }
    }
}

/**
 * @brief Prints current pump status to Serial monitor
 */
void printPumpStatus() {
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    Serial.println("=== DUAL PUMP WITH DEFLATION STATUS ===");
#elif PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    Serial.println("=== SINGLE PUMP INFLATION ONLY STATUS ===");
#endif

    Serial.print("Pump Enabled: ");
    Serial.println(pumpOp.enabled ? "YES" : "NO");
    Serial.print("Mode: ");
    switch (pumpOp.mode) {
        case PUMP_OFF: Serial.println("OFF"); break;
        case PUMP_MANUAL: 
            Serial.print("MANUAL");
            if (pumpOp.duration == 0) {
                Serial.println(" (Continuous)");
            } else {
                Serial.print(" (Duration: ");
                Serial.print(pumpOp.duration / 1000);
                Serial.println(" seconds)");
            }
            break;
        case PUMP_VOC_AUTO: 
            Serial.print("VOC AUTO (Threshold: ");
            Serial.print(vocMonitor.threshold);
            Serial.print(", Duration: ");
            Serial.print(pumpOp.duration / 1000);
            Serial.println(" seconds)");
            break;
        case PUMP_TIMED: 
            Serial.print("TIMED (");
            Serial.printf("%02d:%02d:%02d - %02d:%02d:%02d", 
                         timedOp.startHour, timedOp.startMinute, timedOp.startSecond,
                         timedOp.endHour, timedOp.endMinute, timedOp.endSecond);
            Serial.print(", Duration: ");
            Serial.print(pumpOp.duration / 1000);
            Serial.print(" seconds, Active: ");
            Serial.print(timedOp.isActive ? "YES" : "NO");
            Serial.println(")");
            break;
    }
    Serial.print("Operation: ");
    
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    Serial.println(pumpOp.enabled ? (pumpOp.isInflating ? "INFLATE" : "DEFLATE") : "STOPPED");
#elif PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
    Serial.println(pumpOp.enabled ? "INFLATE ONLY" : "STOPPED");
#endif

    Serial.print("Flow Rate: ");
    Serial.print(pumpOp.flowRate);
    Serial.println(" (0-255)");
    
    if (pumpOp.enabled && pumpOp.duration > 0) {
        unsigned long elapsed = millis() - pumpOp.startTime;
        unsigned long remaining = (pumpOp.duration > elapsed) ? (pumpOp.duration - elapsed) : 0;
        Serial.print("Time Remaining: ");
        Serial.print(remaining / 1000);
        Serial.println(" seconds");
    }
    
    Serial.println("L298N Pin States (Optimized Single Direction):");
    Serial.print("  Pump1 ENA (22): PWM=");
    Serial.println(pumpOp.enabled ? pumpOp.flowRate : 0);
    Serial.print("  Pump1 IN1 (21): ");
    Serial.println(digitalRead(PUMP1_IN1_PIN) ? "HIGH" : "LOW");
    
#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
    Serial.print("  Pump2 ENA (23): PWM=");
    Serial.println(pumpOp.enabled ? pumpOp.flowRate : 0);
    Serial.print("  Pump2 IN1 (19): ");
    Serial.println(digitalRead(PUMP2_IN1_PIN) ? "HIGH" : "LOW");
#endif

    Serial.print("  Valve ENA (18): PWM=");
    Serial.println(digitalRead(VALVE_IN1_PIN) ? "255 (5V)" : "0");
    Serial.print("  Valve IN1 (5): ");
    Serial.println(digitalRead(VALVE_IN1_PIN) ? "HIGH" : "LOW");
    Serial.println("=======================================");
} 