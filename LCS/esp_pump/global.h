#ifndef GLOBAL_H
#define GLOBAL_H

#define TINY_GSM_MODEM_SIM7600 // <<< Adjust if your modem is different
#include <TinyGsmClient.h>     // Required for TinyGsm object
#include <HardwareSerial.h>
#include "BluetoothSerial.h"   // For Bluetooth if needed
#include <BlynkSimpleTinyGSM.h>
#include <TimeLib.h>           // For time functions

// Modem configuration
#define SerialAT Serial1
#define SerialMon Serial

#define MODEM_TX 27
#define MODEM_RX 26
#define MODEM_PWRKEY 4
#define MODEM_DTR 32
#define MODEM_RI 33
#define MODEM_FLIGHT 25
#define MODEM_STATUS 34
#define BAT_ADC 35

// Arduino Mega communication
#define ARDUINO_UART_RX 16    // ESP32 pins for Arduino communication
#define ARDUINO_UART_TX 17

// Bluetooth communication with main ESP32
#define BT_DEVICE_NAME "ESP32_PumpController_BT"
#define MAIN_ESP32_BT_NAME "ESP32_EnvSensor_BT"

// LED Pin
#define LED_PIN 12

// Other constants
#define UART_BAUD 115200
#define ARDUINO_BAUD 9600

// Virtual pin definitions for Blynk Pump Control
#define VPIN_PUMP_ENABLE V0         // Pump ON/OFF switch
#define VPIN_PUMP_FLOW_RATE V1      // Flow rate slider (0-255)
#define VPIN_VOC_AUTO_ENABLE V2     // Auto-trigger by VOC enable
#define VPIN_VOC_THRESHOLD V3       // VOC concentration threshold
#define VPIN_TIMED_START_ENABLE V4  // Timed start enable
#define VPIN_TIMED_START_TIME V5    // Timed start time (minutes from now)
#define VPIN_SAMPLING_TIME V6       // Sampling duration (seconds)
#define VPIN_PUMP_STATUS V7         // Pump status display (read-only)
#define VPIN_CURRENT_VOC V8         // Current VOC reading (read-only)
#define VPIN_PUMP_MODE V9           // Current pump mode (read-only)
#define VPIN_ARDUINO_STATUS V10     // Arduino connection status
#define VPIN_MAIN_ESP32_STATUS V11  // Main ESP32 Bluetooth connection status
#define VPIN_SYSTEM_RESET V12       // System reset button
#define VPIN_EMERGENCY_STOP V13     // Emergency stop button

// Command definitions for Arduino communication
#define CMD_INFO "INFO"           // Check communication status
#define CMD_DATA "DATA"           // Request/send sensor data
#define CMD_ACK "ACK"             // Acknowledgment
#define CMD_FAIL "FAIL"           // Command failed
#define CMD_INFLATE "INFLATE"     // Inflate command
#define CMD_DEFLATE "DEFLATE"     // Deflate command
#define CMD_STOP "STOP"           // Stop command

// Timeout and retry settings
#define BLYNK_CONNECT_TIMEOUT 20000
#define BLYNK_SEND_INTERVAL 5000
#define CMD_TIMEOUT 5000          // Command timeout in milliseconds
#define MAX_RETRIES 3             // Maximum retries for commands
#define WDT_TIMEOUT 60000         // Watchdog timeout in milliseconds

// Pump operation modes
enum PumpMode {
  PUMP_OFF = 0,
  PUMP_MANUAL = 1,
  PUMP_VOC_AUTO = 2,
  PUMP_TIMED = 3,
  PUMP_SAMPLING = 4
};

// Pump operation structure
struct PumpOperation {
  bool enabled;
  int flowRate;           // 0-255
  PumpMode mode;
  unsigned long startTime;
  unsigned long duration; // in milliseconds
  bool isInflating;
};

// VOC monitoring structure
struct VOCMonitoring {
  bool autoEnabled;
  float threshold;
  float currentLevel;
  unsigned long lastCheck;
};

// Timed operation structure
struct TimedOperation {
  bool enabled;
  unsigned long startTime;
  unsigned long samplingDuration;
  bool scheduled;
};

// Global variable declarations
extern TinyGsm modem;
extern HardwareSerial ArduinoSerial;
extern BluetoothSerial MainESP32_BT;
extern volatile bool internetAvailable;
extern bool blynkConnected;

// Pump system globals
extern PumpOperation pumpOp;
extern VOCMonitoring vocMonitor;
extern TimedOperation timedOp;
extern unsigned long lastArduinoComm;
extern bool arduinoConnected;
extern unsigned long lastMainESP32Comm;
extern bool mainESP32Connected;

// Timer and scheduling
extern unsigned long lastStatusUpdate;
extern unsigned long lastVOCCheck;

#endif // GLOBAL_H 