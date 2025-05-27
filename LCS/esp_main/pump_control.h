#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#ifdef ENABLE_PUMP_CONTROL

#include "BluetoothSerial.h" // Required for Bluetooth functionalities

/**
 * @brief Initializes Bluetooth Serial for pump control communication.
 * @return True if initialization was successful, false otherwise.
 */
bool setupPumpControl();

/**
 * @brief Handles incoming commands from the connected Bluetooth device.
 */
void handlePumpControlCommands();

/**
 * @brief Sends current sensor data to pump ESP32 via Bluetooth
 */
void sendSensorDataToPump();

#endif // ENABLE_PUMP_CONTROL

#endif // PUMP_CONTROL_H
