#ifndef GLOBAL_H
#define GLOBAL_H

#define TINY_GSM_MODEM_SIM7600 // <<< Adjust if your modem is different
#include <TinyGsmClient.h>     // Required for TinyGsm object
#include <HardwareSerial.h>
#include <TimeLib.h>           // For time functions

#define PUMP_NUM 1

#define BLYNK_TEMPLATE_ID "TMPL6PNG1glup"
#define BLYNK_TEMPLATE_NAME "Air Pump"

#if PUMP_NUM == 1
#define BLYNK_AUTH_TOKEN "jiH6wNgCtex-XP0jmEBz5iy2DEDvcrOc" // Replace with your token
#elif PUMP_NUM == 2
#define BLYNK_AUTH_TOKEN "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI" // Replace with your token
#elif PUMP_NUM == 3
#define BLYNK_AUTH_TOKEN "tzqMA1jqbtyY2iCwSWi6u34KtkcQKZ0L" // Replace with your token
#elif PUMP_NUM == 4
#define BLYNK_AUTH_TOKEN "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI" // Replace with your token
#endif

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

// Pump control plan selection
#define DUAL_PUMP_WITH_DEFLATION 1        // Dual pump system with inflation and deflation capability
#define SINGLE_PUMP_INFLATION_ONLY 2      // Single pump system with inflation only (no deflation)

// Select pump control plan (change this to switch between configurations)
#define PUMP_CONTROL_PLAN SINGLE_PUMP_INFLATION_ONLY

// Direct pump control pins (ESP32 to L298N motor drivers)
// Optimized pin usage - single direction only (no direction change needed)
#define PUMP1_ENA_PIN 22          // Enable pin for Pump 1 (PWM speed control)
#define PUMP1_IN1_PIN 21          // IN1 pin for Pump 1 (single direction control)

#if PUMP_CONTROL_PLAN == DUAL_PUMP_WITH_DEFLATION
#define PUMP2_ENA_PIN 23          // Enable pin for Pump 2 (PWM speed control)
#define PUMP2_IN1_PIN 19          // IN1 pin for Pump 2 (single direction control)
#endif

// Valve controlled via L298N motor driver for 5V power
#define VALVE_ENA_PIN 18          // Enable pin for Valve (PWM - always full duty cycle for 5V)
#define VALVE_IN1_PIN 5           // IN1 pin for Valve (digital ON/OFF control)

// Single pump inflation-only system timing settings
#if PUMP_CONTROL_PLAN == SINGLE_PUMP_INFLATION_ONLY
#define VALVE_PUMP_START_DELAY 200   // Delay in ms between valve open and pump start
#define PUMP_VALVE_STOP_DELAY 200    // Delay in ms between pump stop and valve close
#endif

// LED Pin
#define LED_PIN 12

// Other constants
#define UART_BAUD 115200

// Virtual pin definitions for Blynk Pump Control
#define VPIN_PUMP_ENABLE V0         // Pump ON/OFF switch
#define VPIN_PUMP_FLOW_RATE V1      // Flow rate slider (0-2.5 LPM)
#define VPIN_PUMP_MODE V2           // Operation mode selector (0=Manual, 1=VOC Auto, 2=Timed)
#define VPIN_SAMPLING_DURATION V3   // Sampling duration for all modes (0=continuous for manual, >0=seconds)
#define VPIN_VOC_THRESHOLD V5       // VOC concentration threshold
#define VPIN_TIMED_TIME_RANGE V8    // Timed operation time range (HH:mm:ss-HH:mm:ss format)
#define VPIN_PUMP_STATUS V11        // Pump status display (read-only)
#define VPIN_CURRENT_VOC V12        // Current VOC reading (read-only)
#define VPIN_MAIN_ESP32_STATUS V13  // Main ESP32 connection status (via API)
#define VPIN_SYSTEM_RESET V14       // System reset button
#define VPIN_EMERGENCY_STOP V15     // Emergency stop button
#define VPIN_VOC_DATA_RECEIVE V16   // VOC data received from main ESP32 (API)

// Command definitions for communication
#define CMD_INFO "INFO"           // Check communication status
#define CMD_DATA "DATA"           // Request/send sensor data
#define CMD_ACK "ACK"             // Acknowledgment
#define CMD_FAIL "FAIL"           // Command failed
#define CMD_VOC_DATA "VOC_DATA"   // VOC data command
#define CMD_RESET "RESET"         // Reset command
#define CMD_NONET "NONET"         // No internet mode

// Timeout and retry settings
#define BLYNK_CONNECT_TIMEOUT 20000
#define BLYNK_SEND_INTERVAL 5000
#define CMD_TIMEOUT 5000          // Command timeout in milliseconds
#define MAX_RETRIES 3             // Maximum retries for commands
#define WDT_TIMEOUT 180000        // Watchdog timeout in milliseconds (3 minutes)

// Memory optimization settings
#define CONFIG_ARDUINOJSON_USE_LONG_LONG 0
#define CONFIG_ARDUINOJSON_USE_DOUBLE 0

// Pump operation modes
enum PumpMode {
  PUMP_OFF = -1,        // Not directly selectable from Blynk (pump disabled state)
  PUMP_MANUAL = 0,      // Blynk mode selector value 0
  PUMP_VOC_AUTO = 1,    // Blynk mode selector value 1
  PUMP_TIMED = 2        // Blynk mode selector value 2
};

// Pump operation structure
struct PumpOperation {
  bool enabled;
  int flowRate;           // 0-255 duty cycle
  PumpMode mode;
  unsigned long startTime;
  unsigned long duration; // in milliseconds (0 = continuous for manual mode)
  bool isInflating;
};

// VOC monitoring structure
struct VOCMonitoring {
  float threshold;
  float currentLevel;
  unsigned long lastCheck;
  unsigned long lastUpdate;
};

// Timed operation structure
struct TimedOperation {
  int startHour;          // Start time hour (0-23)
  int startMinute;        // Start time minute (0-59)
  int startSecond;        // Start time second (0-59)
  int endHour;            // End time hour (0-23)
  int endMinute;          // End time minute (0-59)
  int endSecond;          // End time second (0-59)
  bool isActive;          // Currently within the scheduled time window
  unsigned long lastSampleStart;  // Last time a sample was started
};

// Global variable declarations
extern TinyGsm modem;
extern volatile bool internetAvailable;
extern bool blynkConnected;

// Pump system globals
extern PumpOperation pumpOp;
extern VOCMonitoring vocMonitor;
extern TimedOperation timedOp;
extern unsigned long lastMainESP32Comm;
extern bool mainESP32Connected;

// Timer and scheduling
extern unsigned long lastStatusUpdate;
extern unsigned long lastVOCCheck;

#endif // GLOBAL_H 