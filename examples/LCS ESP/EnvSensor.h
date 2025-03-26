/*
  EnvSensor.h - Header file for Environmental Sensor System
  Adapted from original Arduino Mega code for ESP32 with Blynk cloud integration
*/

#ifndef ENV_SENSOR_H
#define ENV_SENSOR_H

// Include libraries
#include <Wire.h>
#include "rgb_lcd.h"
#include <SPI.h>
#include <SD.h>
#include <Time.h>
#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

// Blynk configuration
#define BLYNK_TEMPLATE_ID "TMPL6kFMi5YBK"
#define BLYNK_TEMPLATE_NAME "EnvSensor"
#define BLYNK_AUTH_TOKEN "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI"  // Replace with your token

// Select modem type
#define TINY_GSM_MODEM_SIM7600

// Modem configuration
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

// Define the maximum file size (in bytes)
const unsigned long MAX_FILE_SIZE = 1000000; // 1 MB

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
void readSensors();
void logToSD();
void reconnectSD();
float readBattery(uint8_t pin);

#endif // ENV_SENSOR_H 