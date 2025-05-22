/*
  =============
  == Summary ==
  =============

  This is the prototype Arduino code for sensor assembly
  It measures inorganic gases (CO, CO2, NO2, O3, SO2), VOCs, temperature, and relative humidity.

  Ming
  Dec 13, 2021
*/

// Include libraries
#include <SoftwareSerial.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <Time.h>
#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ArduinoJson.h>

// Add TimerOne library for hardware timer
#include <TimerOne.h>

// Command definitions for ESP32 communication
#define CMD_INFO "INFO"           // Check communication status
#define CMD_DATA "DATA"           // Request sensor data
#define CMD_RESET "RESET"         // Reset command
#define CMD_ACK "ACK"             // Acknowledgment
#define CMD_CONNECTED "CONNECTED" // Connected to Blynk
#define CMD_NONET "NONET"         // No internet mode query
#define CMD_YES "YES"             // Positive response
#define CMD_NO "NO"               // Negative response
#define CMD_COMPLETE "COMPLETE"   // Data successfully received
#define CMD_FAIL "FAIL"           // Command failed

// Alphasense Sensor Pins
#define pin_CO_w A0  // CO B4 - Working Electrode
#define pin_CO_a A1  // CO B4 - Auxiliary Electrode
#define pin_SO2_w A2 // SO2 - Working Electrode
#define pin_SO2_a A3 // SO2 - Auxiliary Electrode
#define pin_NO2_w A4 // NO2 - Working Electrode
#define pin_NO2_a A5 // NO2 - Auxiliary Electrode
#define pin_OX_w A6  // OX (O3) - Working Electrode
#define pin_OX_a A7  // OX (O3) - Auxiliary Electrode
#define pin_pid A8   // PID (VOCs) - Voltage Output
#define pin_CO2 A9   // CO2 Sensor - Voltage Output

// other constants
const float voltageSupply = 5.0;       // Analog voltage supply (usually 5V for Arduino Uno/Mega)
#define SEALEVELPRESSURE_HPA (1013.25) // Standard sea level pressure in hPa for altitude calculation

// SoftwareSerial instances (Note: SoftwareSerial library not used directly here, likely for ESP32 comms if different pins were used)
rgb_lcd lcd;           // LCD display object
Adafruit_BME680 bme(&Wire); // BME680 sensor object using I2C

// Communication control flags and timing
bool networkStatus = false;                // Tracks if ESP32 reports network connectivity (e.g., Blynk)
bool ESP32Status = false;                  // Tracks if communication with ESP32 is active
bool sensorStatus = false;                 // Tracks if BME680 sensor initialized successfully
unsigned long lastEspCommandTime = 0;      // Timestamp of the last received command from ESP32
unsigned long lastCommunicationCheck = 0; // Timestamp of the last communication check
const unsigned long COMM_TIMEOUT = 60000;  // ESP32 communication timeout (60 seconds)
bool autoResetEnabled = false;             // Flag to enable/disable auto-reset on timeout (currently false)
const int resetPin = 8;                    // Digital pin connected to ESP32 reset circuit (adjust as needed)

// Sensor Readings Variables
float rh = 0;      // Relative Humidity (%)
float t = 0;       // Temperature (Celsius)
float p = 0;       // Pressure (Pascals)
double v_CO_w = 0; // CO Working Electrode Voltage
double v_CO_a = 0; // CO Auxiliary Electrode Voltage
double v_SO2_w = 0; // SO2 Working Electrode Voltage
double v_SO2_a = 0; // SO2 Auxiliary Electrode Voltage
double v_NO2_w = 0; // NO2 Working Electrode Voltage
double v_NO2_a = 0; // NO2 Auxiliary Electrode Voltage
double v_OX_w = 0; // OX (O3) Working Electrode Voltage
double v_OX_a = 0; // OX (O3) Auxiliary Electrode Voltage
double v_pid_w = 0; // PID (VOCs) Voltage
double v_co2_w = 0; // CO2 Sensor Voltage

// Removed Sampling control variables as sampling is driven by ESP32 requests
// unsigned long lastSampleTime = 0;
// const unsigned long SAMPLE_INTERVAL = 10000; // Sample every 10 seconds
// volatile bool dataReady = false;

/**
 * @brief Sets the Arduino's internal time using TimeLib.
 * @param timeStr A string representing the time in "YYYY-MM-DDTHH:MM:SS" format.
 */
void setTimeFromString(String timeStr)
{
  int year = timeStr.substring(0, 4).toInt();
  int month = timeStr.substring(5, 7).toInt();
  int day = timeStr.substring(8, 10).toInt();
  int hour = timeStr.substring(11, 13).toInt();
  int minute = timeStr.substring(14, 16).toInt();
  int second = timeStr.substring(17, 19).toInt();

  setTime(hour, minute, second, day, month, year);
  Serial.print("Time set to: ");
  Serial.println(timeStr);
}

/**
 * @brief Processes commands received from the ESP32 via Serial2.
 *        Parses incoming JSON messages and handles defined commands.
 *        Also checks for communication timeouts.
 */
void processEspCommands()
{
  if (Serial2.available())
  {
    // Wait for a complete JSON object
    String jsonString = "";
    bool foundStart = false;
    bool foundEnd = false;

    // Clear any garbage data first
    while (Serial2.available() && Serial2.peek() != '{')
    {
      Serial2.read();
    }

    // Read until we find a complete JSON object
    unsigned long startTime = millis();
    while (Serial2.available() && (millis() - startTime < 1000))
    {
      char c = Serial2.read();

      if (c == '{')
      {
        foundStart = true;
        jsonString = "{";
        continue;
      }

      if (foundStart)
      {
        jsonString += c;

        if (c == '}')
        {
          foundEnd = true;
          break;
        }
      }
    }

    // Only process if we have a complete JSON object
    if (foundStart && foundEnd)
    {
      Serial.println("Received JSON: " + jsonString);

      // Parse JSON
      JsonDocument cmdDoc;
      DeserializationError error = deserializeJson(cmdDoc, jsonString);

      if (error)
      {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
      }

      // Extract command
      String command = cmdDoc["cmd"].as<String>();

      // Update communication timestamp
      lastEspCommandTime = millis();

      // Handle different commands
      if (command == CMD_INFO)
      {
        // Information/communication check
        sendJsonResponse(CMD_ACK, "");
        ESP32Status = true;
        Serial.println("Info request acknowledged");
      }
      else if (command == CMD_RESET)
      {
        // ESP32 is requesting a reset
        sendJsonResponse(CMD_ACK, "");
        Serial.println("Reset requested by ESP32");
        delay(200);
        resetArduino();
      }
      else if (command == CMD_NONET)
      {
        // ESP32 is asking if we accept No Internet mode
        sendJsonResponse(CMD_YES, "");
        Serial.println("Accepting No Internet mode");
      }
      else if (command == CMD_COMPLETE)
      {
        // ESP32 has successfully received data
        sendJsonResponse(CMD_ACK, "");
        Serial.println("ESP32 confirmed data reception");
      }
      else if (command == CMD_CONNECTED)
      {
        // ESP32 connected to Blynk
        networkStatus = true;
        sendJsonResponse(CMD_ACK, "");
        Serial.println("ESP32 connected to Blynk");
      }
      else if (command == CMD_DATA)
      {
        // ESP32 is requesting data
        // sendJsonResponse(CMD_ACK, "");
        Serial.println("ESP32 requesting data");
        sampleSensors();
      }
    }
  }

  // Check for ESP32 communication timeout every 5 seconds
  if (millis() - lastCommunicationCheck > 5000)
  {
    lastCommunicationCheck = millis();

    // Check for timeout only if auto-reset is enabled
    if (millis() - lastEspCommandTime > COMM_TIMEOUT)
    {
      if (ESP32Status) { // Only print timeout message once
        Serial.println("ESP32 communication timeout.");
        ESP32Status = false; // Mark ESP32 as disconnected
      }

      if (autoResetEnabled && (millis() - lastEspCommandTime > COMM_TIMEOUT))
      {
        Serial.println("ESP32 communication timeout, resetting...");
        resetArduino();
      }
    }
  }
}

/**
 * @brief Sends a JSON formatted response to the ESP32 via Serial2.
 * @param cmd The command string (e.g., CMD_ACK, CMD_DATA).
 * @param data The data payload string (can be empty).
 */
void sendJsonResponse(const String &cmd, const String &data)
{
  JsonDocument respDoc;
  respDoc["cmd"] = cmd;
  respDoc["data"] = data;

  String response;
  serializeJson(respDoc, response);
  Serial2.println(response);

  Serial.print("Sent response: ");
  Serial.println(response);
}

// Sample sensors on a timer
void sampleSensors()
{
  // This function is called by the timer interrupt
  // Use noInterrupts/interrupts to prevent other interrupts during critical sections
  // Read sensor data
  v_CO_w = analogRead(pin_CO_w) / 1024.000 * voltageSupply;
  v_CO_a = analogRead(pin_CO_a) / 1024.000 * voltageSupply;
  v_SO2_w = analogRead(pin_SO2_w) / 1024.000 * voltageSupply;
  v_SO2_a = analogRead(pin_SO2_a) / 1024.000 * voltageSupply;
  v_NO2_w = analogRead(pin_NO2_w) / 1024.000 * voltageSupply;
  v_NO2_a = analogRead(pin_NO2_a) / 1024.000 * voltageSupply;
  v_OX_w = analogRead(pin_OX_w) / 1024.000 * voltageSupply;
  v_OX_a = analogRead(pin_OX_a) / 1024.000 * voltageSupply;
  v_pid_w = analogRead(pin_pid) / 1024.000 * voltageSupply;
  v_co2_w = analogRead(pin_CO2) / 1024.000 * voltageSupply;

  // Get BME680 readings
  if (bme.performReading())
  {
    rh = bme.humidity;   // relative humidity (%)
    t = bme.temperature; // temperature (C)
    p = bme.pressure;    // pressure (pa)
  }
  sendSensorDataToEsp();
}

/**
 * @brief Creates a JSON object containing the latest sensor readings
 *        and sends it to the ESP32 via Serial2.
 */
void sendSensorDataToEsp()
{
  // Create JSON document with an array for data
  JsonDocument sensorDoc;

  // Format time string
  char timebuffer_save[25];
  sprintf(timebuffer_save, "%02d %02d %02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());

  // Create the command structure
  sensorDoc["cmd"] = CMD_DATA;

  // Create a data array to store values in a consistent order
  JsonArray dataArray = sensorDoc.createNestedArray("data");

  // Add all data to the array in a predefined order
  dataArray.add(timebuffer_save);            // 0: timestamp (YYYY MM DD HH:MM:SS format)
  dataArray.add(t);                                      // 1: temperature
  dataArray.add(rh);                                     // 2: humidity
  dataArray.add(p);                                      // 3: pressure
  dataArray.add(bme.readAltitude(SEALEVELPRESSURE_HPA)); // 4: altitude
  dataArray.add(v_CO_w);                                 // 5: CO_w
  dataArray.add(v_CO_a);                                 // 6: CO_a
  dataArray.add(v_SO2_w);                                // 7: SO2_w
  dataArray.add(v_SO2_a);                                // 8: SO2_a
  dataArray.add(v_NO2_w);                                // 9: NO2_w
  dataArray.add(v_NO2_a);                                // 10: NO2_a
  dataArray.add(v_OX_w);                                 // 11: OX_w
  dataArray.add(v_OX_a);                                 // 12: OX_a
  dataArray.add(v_pid_w);                                // 13: pid_w
  dataArray.add(v_co2_w);                                // 14: CO2_w

  // Serialize and send the complete command
  serializeJson(sensorDoc, Serial2);

  Serial.println("Sensor data sent to ESP32");
}

/**
 * @brief Resets the Arduino.
 *        Attempts to send a reset notification to ESP32 first.
 *        Uses a hardware pin toggle followed by a software reset as a fallback.
 */
void resetArduino()
{
  Serial.println("Resetting Arduino...");

  // Try to notify ESP32 before reset
  sendJsonResponse(CMD_RESET, "");

  delay(100);
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);

  // If hardware reset doesn't work, use software reset
  asm volatile("jmp 0"); // Software reset
}

/**
 * @brief Updates the LCD display with current time and status information.
 *        Shows ESP32 connection status, Network status, and Sensor status.
 *        Changes LCD backlight color based on status.
 */
void displayStatus()
{
  lcd.clear();

  // First line: Time and ESP32 status
  lcd.setCursor(0, 0);
  char timebuffer[10];
  sprintf(timebuffer, "%02d:%02d", hour(), minute());
  lcd.print(timebuffer);

  lcd.setCursor(8, 0);
  if (ESP32Status)
  {
    lcd.print("ESP32:OK");
    lcd.setRGB(23, 180, 36); // Green for good
  }
  else
  {
    lcd.print("ESP32:--");
    lcd.setRGB(255, 165, 0); // Orange for warning
  }

  // Second line: Network and Sensor status
  lcd.setCursor(0, 1);
  if (networkStatus)
  {
    lcd.print("NET:OK ");
  }
  else
  {
    lcd.print("NET:-- ");
  }

  if (sensorStatus)
  {
    lcd.print("SENS:OK");
    lcd.setRGB(23, 180, 36); // Green for good
  }
  else
  {
    lcd.print("SENS:--");
    lcd.setRGB(255, 0, 0); // Red for error
  }
}

/**
 * @brief Initializes hardware, sensors, communication, and display.
 *        Runs once at startup.
 */
void setup()
{
  // begin serial communication
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(500);
  Serial.println("Initializing");

  // Initialize reset pin
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);

  // Update communication timestamp
  lastEspCommandTime = millis();

  // initiate LCD monitor
  lcd.begin(16, 2);

  // GREEN for being good
  int colorR = 23;
  int colorG = 180;
  int colorB = 36;

  lcd.setRGB(colorR, colorG, colorB);
  lcd.setCursor(0, 0);
  lcd.print("LCD module ready");
  delay(500);

  Serial.println("LCD monitor initialized");
  delay(500);
  Serial.println("1");

  // Initialize BME680 sensor
  if (!bme.begin())
  {
    Serial.println("BME680 error!");
    while (1)
      ;
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  lcd.setCursor(0, 1);
  lcd.print("BME ready");
  sensorStatus = true;
  delay(500);

  Serial.println("Sensors initialized");
  delay(500);

  // starting measurement loop
  Serial.println("Connecting to AP...");

  // Perform initial sensor reading
  // sampleSensors();

  // Display initial time on LCD briefly if needed, but filename is not used elsewhere
  char timebuffer_init[20];
  sprintf(timebuffer_init, "%02d%02d%02d %02d:%02d:%02d", year() % 100, month(), day(), hour(), minute(), second());
  lcd.setCursor(0, 1); // 2nd row, 1st col
  lcd.print(timebuffer_init);
  Serial.print("Initial Time: ");
  Serial.println(timebuffer_init);

  lcd.clear();
  lcd.setCursor(0, 0);
  // Print headers to serial for debugging
  Serial.print("Time");
  Serial.print(" ");
  Serial.print("Temperature(C)");
  Serial.print(" ");
  Serial.print("Humidity(%)");
  Serial.print(" ");
  Serial.print("Pressure(pa)");
  Serial.print(" ");
  Serial.print("CO_W");
  Serial.print(" ");
  Serial.print("CO_A");
  Serial.print(" ");
  Serial.print("SO2_W");
  Serial.print(" ");
  Serial.print("SO2_A");
  Serial.print(" ");
  Serial.print("NO2_W");
  Serial.print(" ");
  Serial.print("NO2_A");
  Serial.print(" ");
  Serial.print("OX_W");
  Serial.print(" ");
  Serial.print("OX_A");
  Serial.print(" ");
  Serial.print("PID_W");
  Serial.print(" ");
  Serial.print("CO2_W");
  Serial.print(" ");
  Serial.println(" ");
  // record sensor readings
  lcd.print("Start recording");
  delay(1000);
  lcd.clear();
  lcd.print("3");
  delay(1000);
  lcd.print("2");
  delay(1000);
  lcd.print("1");
  delay(1000);
  lcd.clear();
}

/**
 * @brief Main loop. Continuously processes ESP32 commands and updates the LCD status.
 */
void loop()
{
  // Process any incoming commands from ESP32
  processEspCommands();
  // Display status information
  displayStatus();

  // Small delay to prevent LCD flickering
  // delay(1000);
}
