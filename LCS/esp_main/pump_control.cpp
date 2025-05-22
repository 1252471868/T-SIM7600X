#include "global.h" // Include first for the macro definition

#ifdef ENABLE_PUMP_CONTROL

#include "pump_control.h"
#include "global.h"       // For ESP_BT extern declaration and Serial logging
#include "communication.h" // For processIncomingCommands

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
      Serial.println("Pump Control Bluetooth initialized. Ready to pair.");
      return true;
    }
}

/**
 * @brief Handles incoming commands from the connected Bluetooth device.
 */
void handlePumpControlCommands() {
    // Process incoming commands from second ESP32 (Bluetooth)

}

#endif // ENABLE_PUMP_CONTROL
