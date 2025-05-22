#include "global.h"

// Modem Object (conditional compilation for debugging)
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial); // Note: Using Serial for debug output
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT); // TinyGSM modem object
#endif

volatile bool internetAvailable = true; // Network availability status

// Define global variables specific to Communication module
unsigned long lastCommTime = 0; // TODO: Still needs thought for multiple devices
float temperature = 0.0, humidity = 0.0, pressure = 0.0;
double v_CO_w = 0.0, v_CO_a = 0.0, v_SO2_w = 0.0, v_SO2_a = 0.0;
double v_NO2_w = 0.0, v_NO2_a = 0.0, v_OX_w = 0.0, v_OX_a = 0.0;
double v_pid_w = 0.0, v_co2_w = 0.0;

// Hardware Communication Objects
HardwareSerial ArduinoSerial(2); // UART2 (GPIO 16=RX, 17=TX) for Arduino communication

BluetoothSerial ESP_BT; // <<< Added Bluetooth Serial object

volatile bool stopReading = false;                   // Defined here now
volatile bool autoResetEnabled = AUTO_RESET_ENABLED; // Assuming AUTO_RESET_ENABLED in EnvSensor.h
bool blynkConnected = false;                         // Defined here now