/*
  EnvSensor.h - Header file for Environmental Sensor System
  Adapted from original Arduino Mega code for ESP32 with Blynk cloud integration
*/

#ifndef ENV_SENSOR_H
#define ENV_SENSOR_H

// Include libraries
#include <Wire.h>
// #include "rgb_lcd.h"
#include <SPI.h>
#include <SD.h>
#include <Time.h>
#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include <ArduinoJson.h>

#define BOX_NUM   4

// Blynk configuration
#define BLYNK_TEMPLATE_ID "TMPL6kFMi5YBK"
#define BLYNK_TEMPLATE_NAME "EnvSensor"
#if BOX_NUM == 1
#define BLYNK_AUTH_TOKEN "iihKlmC4B_tYYOZZS68Fm9H8PUJX7Ed_" // Replace with your token
#elif BOX_NUM == 2
#define BLYNK_AUTH_TOKEN "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI"  // Replace with your token
#elif BOX_NUM == 3
#define BLYNK_AUTH_TOKEN "tzqMA1jqbtyY2iCwSWi6u34KtkcQKZ0L" // Replace with your token
#elif BOX_NUM == 4
#define BLYNK_AUTH_TOKEN "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI"  // Replace with your token

#endif

// Select modem type
#define TINY_GSM_MODEM_SIM7600

// Modem configuration
#define SerialAT Serial1
#define SerialMon Serial

#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34
#define BAT_ADC             35

// SD Card configuration
#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13

// Arduino UART configuration
#define ARDUINO_UART_RX 18
#define ARDUINO_UART_TX 19

// LED Pin
#define LED_PIN             12

// Alphasense sensor pins (adjust according to ESP32 analog inputs)
#define pin_CO_w 36  // working electrode for CO B4
#define pin_CO_a 37  // auxiliary electrode for CO B4
#define pin_SO2_w 38 // working, SO2
#define pin_SO2_a 39 // auxiliary, SO2
#define pin_NO2_w 32 // working, NO2
#define pin_NO2_a 33 // auxiliary, NO2
#define pin_OX_w 34  // working, OX
#define pin_OX_a 35  // auxiliary, OX
#define pin_pid 25   // voltage, PID
#define pin_CO2 26   // voltage, CO2

// Other constants
#define UART_BAUD 115200
#define SEALEVELPRESSURE_HPA (1013.25)
const float voltageSupply = 5.0; // voltages
const long minute2run = 15;       // minutes in each loop
const long numberOfLoop = 100000; // number of loops to run

// Virtual pin definitions for Blynk
#define VPIN_TEMP V0             // Temperature
#define VPIN_PRESSURE V1         // Pressure
#define VPIN_HUMIDITY V4         // Humidity
#define VPIN_BATTERY V2          // Battery Percentage
#define VPIN_SD_LOGGING V3       // SD Logging toggle (BLYNK_WRITE)
#define VPIN_CO_W V5             // CO Working Electrode
#define VPIN_CO_A V11             // CO Auxiliary Electrode
#define VPIN_SO2_W V6            // SO2 Working Electrode
#define VPIN_SO2_A V12            // SO2 Auxiliary Electrode
#define VPIN_NO2_W V7            // NO2 Working Electrode
#define VPIN_NO2_A V13            // NO2 Auxiliary Electrode
#define VPIN_OX_W V8            // OX Working Electrode
#define VPIN_OX_A V14            // OX Auxiliary Electrode
#define VPIN_PID_W V9           // PID Working Electrode
#define VPIN_CO2_W V10           // CO2 Working Electrode 
#define VPIN_RESET V17           // Reset toggle (BLYNK_WRITE)
#define VPIN_AUTO_RESET V16      // Auto-Reset toggle (BLYNK_WRITE)
#define VPIN_ARDUINO_STATUS V15  // Arduino connection status (1=OK, 0=Fail)
#define VPIN_SD_STATUS V18       // SD card status (1=OK, 0=Fail)

// Define the maximum file size (in bytes)
const unsigned long MAX_FILE_SIZE = 1000000; // 1 MB

// Command definitions for Arduino-ESP32 communication
#define CMD_INFO        "INFO"     // Check communication status
#define CMD_DATA        "DATA"     // Request sensor data
#define CMD_RESET       "RESET"    // Reset command
#define CMD_ACK         "ACK"      // Acknowledgment
#define CMD_CONNECTED   "CONNECTED" // Connected to Blynk
#define CMD_NONET       "NONET"    // No internet mode query
#define CMD_YES         "YES"      // Positive response
#define CMD_NO          "NO"       // Negative response
#define CMD_COMPLETE    "COMPLETE" // Data successfully received
#define CMD_FAIL        "FAIL"     // Command failed

// Timeout and retry settings
#define BLYNK_CONNECT_TIMEOUT 20000
#define BLYNK_SEND_INTERVAL 10000
#define CMD_TIMEOUT     5000       // Command timeout in milliseconds
#define MAX_RETRIES     5          // Maximum retries for commands
#define COMM_CHECK_MS   1000       // Communication check interval

// Auto-reset control
#define AUTO_RESET_ENABLED true    // Enable auto reset by default

// GPRS credentials
extern char apn[];
extern char user[];
extern char pass[];
extern char auth[];

// Function prototypes
void sendSensorData();
void setupModem();
void setupBlynk();
void setupSD();
void setupTime();
void readSensorsFromArduino();
void logToSD();
void reconnectSD();
float readBattery(uint8_t pin);

// Communication protocol functions
bool sendCommand(const char* cmd, const String& data, unsigned long timeout = CMD_TIMEOUT);
bool sendCommandWithRetry(const char* cmd, const String& data, int maxRetries = MAX_RETRIES);
void processIncomingCommands();
void resetSystem();
bool checkNoInternetMode();
void reconnectInternetBlynk();

#endif // ENV_SENSOR_H 