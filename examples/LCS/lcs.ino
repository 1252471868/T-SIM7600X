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
#define CMD_INFO        "INFO"     // Check communication status
#define CMD_DATA        "DATA"     // Request sensor data
#define CMD_RESET       "RESET"    // Reset command
#define CMD_ACK         "ACK"      // Acknowledgment
#define CMD_NONET       "NONET"    // No internet mode query
#define CMD_YES         "YES"      // Positive response
#define CMD_NO          "NO"       // Negative response
#define CMD_COMPLETE    "COMPLETE" // Data successfully received
#define CMD_FAIL        "FAIL"     // Command failed

// Alphasense
#define pin_CO_w A0  // working electrode for CO B4
#define pin_CO_a A1  // auxiliary electrode for CO B4
#define pin_SO2_w A2 // working, SO2
#define pin_SO2_a A3 // auxiliary, SO2
#define pin_NO2_w A4 // working, NO2
#define pin_NO2_a A5 // auxiliary, NO2
#define pin_OX_w A6  // working, OX
#define pin_OX_a A7  // auxiliary, OX
#define pin_pid A8   // voltage, PID
#define pin_CO2 A9   // voltage, CO2

// other constants
const float voltageSupply = 5.0; // voltages
#define SEALEVELPRESSURE_HPA (1013.25)

// SoftwareSerial instances
rgb_lcd lcd;
// Adafruit_BME680 bme;
Adafruit_BME680 bme(&Wire);
// Sampling duration - removing these as Arduino will run continuously
char filename[20]; // Keep this for display purposes only

// Communication control
unsigned long lastEspCommandTime = 0;   // Last time we received a command from ESP32
unsigned long lastCommunicationCheck = 0; // Last time we checked communication
const unsigned long COMM_TIMEOUT = 60000; // 60 seconds timeout
bool autoResetEnabled = true;
const int resetPin = 8;  // Digital pin connected to reset circuit (adjust as needed)

// Add timer-based sampling and volatile variables
volatile float rh = 0;
volatile float t = 0;
volatile float p = 0;
volatile double v_CO_w = 0;
volatile double v_CO_a = 0;
volatile double v_SO2_w = 0;
volatile double v_SO2_a = 0;
volatile double v_NO2_w = 0;
volatile double v_NO2_a = 0;
volatile double v_OX_w = 0;
volatile double v_OX_a = 0;
volatile double v_pid_w = 0;
volatile double v_co2_w = 0;

// Sampling control
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 10000; // Sample every 10 seconds
volatile bool dataReady = false;

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

// Process commands received from ESP32
void processEspCommands() {
  if (Serial2.available()) {
    // Wait for a complete JSON object
    String jsonString = "";
    bool foundStart = false;
    bool foundEnd = false;
    
    // Clear any garbage data first
    while (Serial2.available() && Serial2.peek() != '{') {
      Serial2.read();
    }
    
    // Read until we find a complete JSON object
    unsigned long startTime = millis();
    while (Serial2.available() && (millis() - startTime < 1000)) {
      char c = Serial2.read();
      
      if (c == '{') {
        foundStart = true;
        jsonString = "{";
        continue;
      }
      
      if (foundStart) {
        jsonString += c;
        
        if (c == '}') {
          foundEnd = true;
          break;
        }
      }
    }
    
    // Only process if we have a complete JSON object
    if (foundStart && foundEnd) {
      Serial.println("Received JSON: " + jsonString);
      
      // Parse JSON
      JsonDocument cmdDoc;
      DeserializationError error = deserializeJson(cmdDoc, jsonString);
      
      if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
      }
      
      // Extract command
      String command = cmdDoc["cmd"].as<String>();
      
      // Update communication timestamp
      lastEspCommandTime = millis();
      
      // Handle different commands
      if (command == CMD_INFO) {
        // Information/communication check
        sendJsonResponse(CMD_ACK, "");
        Serial.println("Info request acknowledged");
      }
      else if (command == CMD_DATA) {
        // ESP32 is requesting sensor data
        sendJsonResponse(CMD_ACK, "");
        
        // Check if data is ready
        if (dataReady) {
          sendSensorDataToEsp();
        } else {
          // If data isn't ready, we'll just wait for the next timer interrupt
          Serial.println("Data not ready yet, waiting for next sample");
        }
      }
      else if (command == CMD_RESET) {
        // ESP32 is requesting a reset
        sendJsonResponse(CMD_ACK, "");
        Serial.println("Reset requested by ESP32");
        delay(200);
        resetArduino();
      }
      else if (command == CMD_NONET) {
        // ESP32 is asking if we accept No Internet mode
        sendJsonResponse(CMD_YES, "");
        Serial.println("Accepting No Internet mode");
      }
      else if (command == CMD_COMPLETE) {
        // ESP32 has successfully received data
        Serial.println("ESP32 confirmed data reception");
      }
    }
  }
  
  // Check for ESP32 communication timeout every 5 seconds
  if (millis() - lastCommunicationCheck > 5000) {
    lastCommunicationCheck = millis();
    
    if (autoResetEnabled && (millis() - lastEspCommandTime > COMM_TIMEOUT)) {
      Serial.println("ESP32 communication timeout, resetting...");
      resetArduino();
    }
  }
}

// Send JSON formatted response
void sendJsonResponse(const String& cmd, const String& data) {
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
void sampleSensors() {
  // This function is called by the timer interrupt
  // Use noInterrupts/interrupts to prevent other interrupts during critical sections
  noInterrupts();
  
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
  if (bme.performReading()) {
    rh = bme.humidity;   // relative humidity (%)
    t = bme.temperature; // temperature (C)
    p = bme.pressure;    // pressure (pa)
    
    dataReady = true;
  }
  
  interrupts();
}

// Send sensor data to ESP32 in JSON format
void sendSensorDataToEsp() {
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
  dataArray.add(timebuffer_save);     // 0: timestamp
  dataArray.add(t);                   // 1: temperature
  dataArray.add(rh);                  // 2: humidity
  dataArray.add(p);                   // 3: pressure 
  dataArray.add(bme.readAltitude(SEALEVELPRESSURE_HPA)); // 4: altitude
  dataArray.add(v_CO_w);              // 5: CO_w
  dataArray.add(v_CO_a);              // 6: CO_a
  dataArray.add(v_SO2_w);             // 7: SO2_w
  dataArray.add(v_SO2_a);             // 8: SO2_a
  dataArray.add(v_NO2_w);             // 9: NO2_w
  dataArray.add(v_NO2_a);             // 10: NO2_a
  dataArray.add(v_OX_w);              // 11: OX_w
  dataArray.add(v_OX_a);              // 12: OX_a
  dataArray.add(v_pid_w);             // 13: pid_w
  dataArray.add(v_co2_w);             // 14: CO2_w
  
  // Serialize and send the complete command
  String jsonData;
  serializeJson(sensorDoc, jsonData);
  Serial2.println(jsonData);
  
  Serial.println("Sensor data sent to ESP32");
}

// Reset the Arduino
void resetArduino() {
  Serial.println("Resetting Arduino...");
  
  // Try to notify ESP32 before reset
  sendJsonResponse(CMD_RESET, "");
  
  delay(100);
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  
  // If hardware reset doesn't work, use software reset
  asm volatile ("jmp 0");  // Software reset
}

// Check communication with ESP32 on startup - replace existing verification
void checkEspCommunication() {
  bool communicationEstablished = false;
  unsigned long startTime = millis();
  Serial.println("Checking ESP32 communication...");
  
  // Send reset notification
  sendJsonResponse(CMD_RESET, "");
  
  // Try to establish communication for up to 60 seconds
  while (millis() - startTime < 60000) {
    if (Serial2.available()) {
      // Wait for a complete JSON object
      String jsonString = "";
      bool foundStart = false;
      bool foundEnd = false;
      
      // Clear any garbage data first
      while (Serial2.available() && Serial2.peek() != '{') {
        Serial2.read();
      }
      
      // Read until we find a complete JSON object
      unsigned long readStartTime = millis();
      while (Serial2.available() && (millis() - readStartTime < 1000)) {
        char c = Serial2.read();
        
        if (c == '{') {
          foundStart = true;
          jsonString = "{";
          continue;
        }
        
        if (foundStart) {
          jsonString += c;
          
          if (c == '}') {
            foundEnd = true;
            break;
          }
        }
      }
      
      // Only process if we have a complete JSON object
      if (foundStart && foundEnd) {
        Serial.println("Received JSON: " + jsonString);
        
        // Parse JSON
        JsonDocument respDoc;
        DeserializationError error = deserializeJson(respDoc, jsonString);
        
        if (error) {
          Serial.print("JSON parsing failed: ");
          Serial.println(error.c_str());
          continue;
        }
        
        // Extract command
        String response = respDoc["cmd"].as<String>();
        
        if (response == CMD_ACK || response == CMD_INFO) {
          sendJsonResponse(CMD_ACK, "");
          communicationEstablished = true;
          Serial.println("Communication with ESP32 established!");
          lcd.setCursor(0, 1);
          lcd.print("ESP32 Connected");
          lastEspCommandTime = millis();
          break;
        }
      }
    }
    
    // Periodically send reset notification
    if (millis() - startTime > 5000 && (millis() - startTime) % 5000 < 100) {
      sendJsonResponse(CMD_RESET, "");
    }
    
    delay(100);
  }
  
  if (!communicationEstablished) {
    Serial.println("Failed to establish communication with ESP32, resetting...");
    lcd.setCursor(0, 1);
    lcd.print("ESP32 Failed!");
    delay(3000);
    resetArduino();
  }
}

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
    delay(3000);

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
    delay(1000);

    Serial.println("Sensors initialized");
    delay(500);
    Serial.println("2");

    delay(1000);
    // starting measurement loop
    Serial.println("Connecting to AP...");
    
    // Initialize hardware timer for sensor sampling (every 5 seconds)
    Timer1.initialize(SAMPLE_INTERVAL * 1000); // Timer uses microseconds
    Timer1.attachInterrupt(sampleSensors);
    
    // Perform initial sensor reading
    sampleSensors();
    
    // Verify communication with ESP32 using command system
    checkEspCommunication();
    
    sprintf(filename, "%02d%02d%02d%02d%02d%02d.txt", year() % 100, month(), day(), hour(), minute(), second());
    lcd.setCursor(0, 1); // 2nd row, 1st col
    lcd.print(filename); 
    Serial.print(filename);

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

JsonDocument doc;
String jsonData;
void loop()
{
    // Process any incoming commands from ESP32
    processEspCommands();
    
    char timebuffer_save[25];
    char timebuffer[10];
    char number2display[50]; // number to display (10 * original voltage) on LCD monitor

    // Display sensor values on LCD - use the volatile variables
    // that are updated by the timer

    /*
       LCD screen display
    */
    long counter = 0;
    while (counter < 2)
    {
        // Process ESP32 commands while in display loop to maintain responsiveness
        processEspCommands();
        
        lcd.clear();

        lcd.setCursor(0, 0);
        sprintf(timebuffer_save, "%02d %02d %02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
        sprintf(timebuffer, "%02d:%02d:%02d", hour(), minute(), second());
        lcd.print(timebuffer);
        lcd.print(' ');
        lcd.print(int(t)); // Use the volatile variables updated by the timer
        lcd.print(' ');
        lcd.print(int(rh)); // Use the volatile variables updated by the timer
        lcd.print(' ');

        // Use the volatile variables from the timer for display
        if (counter == 0)
        {
            lcd.print('W'); // indicate it is for working voltage
            sprintf(number2display, "%02d %02d %02d %02d %03d", 
                    int(v_CO_w * 10), int(v_SO2_w * 10), int(v_NO2_w * 10), 
                    int(v_OX_w * 10), int(v_pid_w * 100));
        }
        else
        {
            lcd.print('A'); // indicate it is for auxiliary voltage
            sprintf(number2display, "%02d %02d %02d %02d %03d", 
                    int(v_CO_a * 10), int(v_SO2_a * 10), int(v_NO2_a * 10), 
                    int(v_OX_a * 10), int(v_co2_w * 100));
        }
        lcd.setCursor(0, 1); // 2nd line, 1st row
        lcd.print(number2display);

        delay(1000);
        counter++;
    }

    delay(1000); // Reduced from 5000 to allow more frequent checking for commands
}
