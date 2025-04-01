/*
  ESP32 Environmental Monitoring System with Blynk Cloud Integration
  
  This code combines the sensor reading logic from the original Arduino code
  with remote connectivity through Blynk on ESP32.
  
  Features:
  - Gas sensors reading (CO, CO2, NO2, O3, SO2)
  - Temperature, humidity, pressure reading
  - SD card logging
  - Real-time clock synchronization
  - Remote control through Blynk dashboard
*/

#include "EnvSensor.h"
#include <ArduinoJson.h>

#define DUMP_AT_COMMANDS

// Blynk & GPRS credentials
char auth[] = BLYNK_AUTH_TOKEN;
char apn[] = "";  // Your APN
char user[] = ""; // APN username if any
char pass[] = ""; // APN password if any

// Global objects
rgb_lcd lcd;
Adafruit_BME680 bme(&Wire);
BlynkTimer timer;
File SDstorage;
String filename = "DAT";
volatile bool stopReading = false;

// For debugging
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(Serial1);
#endif

// Sensor data variables
float temperature, humidity, pressure;
double v_CO_w, v_CO_a, v_SO2_w, v_SO2_a;
double v_NO2_w, v_NO2_a, v_OX_w, v_OX_a;
double v_pid_w, v_co2_w;

// Remote control of SD card logging via Blynk
BLYNK_WRITE(V3) {
  if (param.asInt() == 1) {
    stopReading = false;
    Serial.println("SD card logging started from Blynk");
    Blynk.logEvent("logging_started"); // Send event notification
  } else {
    stopReading = true;
    Serial.println("SD card logging stopped from Blynk");
    Blynk.logEvent("logging_stopped"); // Send event notification
  }
}

// Synchronize states when Blynk connects
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V3);  // Sync the current state of V3
}

// Read battery voltage
float readBattery(uint8_t pin) {
  int vref = 1100;
  uint16_t volt = analogRead(pin);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

// Add these global variables
StaticJsonDocument<200> doc;
HardwareSerial ArduinoSerial(2); // Using UART2 for Arduino communication

// Replace readSensors() with this new function
void readSensorsFromArduino() {
  if (ArduinoSerial.available()) {
    String jsonData = ArduinoSerial.readStringUntil('\n');
    
    // Deserialize JSON
    DeserializationError error = deserializeJson(doc, jsonData);
    
    if (error) {
      Serial.println("JSON parsing failed!");
      return;
    }

    // Extract values from JSON
    temperature = doc["temperature"];
    humidity = doc["humidity"];
    pressure = doc["pressure"];
    v_CO_w = doc["CO_w"];
    v_CO_a = doc["CO_a"];
    v_SO2_w = doc["SO2_w"];
    v_SO2_a = doc["SO2_a"];
    v_NO2_w = doc["NO2_w"];
    v_NO2_a = doc["NO2_a"];
    v_OX_w = doc["OX_w"];
    v_OX_a = doc["OX_a"];
    v_pid_w = doc["pid_w"];
    v_co2_w = doc["CO2_w"];
  }
}

// Update sendSensorData() function
void sendSensorData() {
  // Read data from Arduino
  readSensorsFromArduino();
  
  // Send to Blynk
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, ((readBattery(BAT_ADC) / 4200) * 100));
  
  // Gas sensors
  Blynk.virtualWrite(V4, v_CO_w);
  Blynk.virtualWrite(V5, v_CO_a);
  Blynk.virtualWrite(V6, v_SO2_w);
  Blynk.virtualWrite(V7, v_SO2_a);
  Blynk.virtualWrite(V8, v_NO2_w);
  Blynk.virtualWrite(V9, v_NO2_a);
  Blynk.virtualWrite(V10, v_OX_w);
  Blynk.virtualWrite(V11, v_OX_a);
  Blynk.virtualWrite(V12, v_pid_w);
  Blynk.virtualWrite(V13, v_co2_w);
  
  // Log to SD if enabled
  if (!stopReading) {
    logToSD();
  }
}

// Set up system time from network
void setupTime() {
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int min = 0;
  int sec = 0;
  float timezone = 0;
  
  // Try to get network time
  for (int8_t i = 5; i; i--) {
    Serial.println("Requesting current network time");
    if (modem.getNetworkTime(&year, &month, &day, &hour, &min, &sec, &timezone)) {
      Serial.printf("Date: %d-%02d-%02d Time: %02d:%02d:%02d\n", 
                    year, month, day, hour, min, sec);
      Serial.printf("Timezone: %.1f\n", timezone);
      
      // Set the system time
      setTime(hour, min, sec, day, month, year);
      break;
    } else {
      Serial.println("Couldn't get network time, retrying in 10s.");
      delay(10000);
    }
  }
}

// Setup SD card
void setupSD() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    lcd.setCursor(0, 0);
    lcd.print("SD fail!");
    return;
  }
  
  Serial.println("SD card initialized");
  lcd.setCursor(0, 0);
  lcd.print("SD ready");
  
  // Find available filename
  bool file = false;
  int count = 0;
  while (!file) {
    if (SD.exists(filename + ".txt")) {
      count++;
      filename = "DAT" + String(count);
    } else {
      file = true;
    }
  }
  
  filename = filename + ".txt";
  Serial.println("Using filename: " + filename);
  
  // Create header in the file
  SDstorage = SD.open(filename, FILE_WRITE);
  if (SDstorage) {
    SDstorage.println("Time Temperature(C) Humidity(%) Pressure(pa) CO_W CO_A SO2_W SO2_A NO2_W NO2_A OX_W OX_A PID_W CO2_W");
    SDstorage.flush();
    SDstorage.close();
  }
}

// Reconnect to SD card if needed
void reconnectSD() {
  while (!SD.begin(SD_CS)) {
    lcd.setCursor(0, 0);
    lcd.print("SD reconnecting");
    delay(500);
  }
  lcd.print("SD reconnected");
}

// Setup the modem
void setupModem() {
  Serial1.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  
  // Setup modem pins
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300);
  digitalWrite(MODEM_PWRKEY, LOW);
  
  pinMode(MODEM_FLIGHT, OUTPUT);
  digitalWrite(MODEM_FLIGHT, HIGH);
  
  Serial.println("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, proceeding anyway");
  }
  
  String name = modem.getModemName();
  Serial.println("Modem Name: " + name);
}

void setup() {
  // Initialize serial console
  Serial.begin(UART_BAUD);
  delay(1000);
  Serial.println("ESP32 Environmental Sensor System Starting...");
  
  // Initialize LCD display
  lcd.begin(16, 2);
  lcd.setRGB(23, 180, 36); // Green
  lcd.setCursor(0, 0);
  lcd.print("System starting");
  
  // Initialize indicator LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize UART for Arduino communication
  ArduinoSerial.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17 (adjust pins as needed)
  
  // Initialize modem
  setupModem();
  
  // Setup SD card
  setupSD();
  
  // Get and set time from network
  setupTime();
  
  // Initialize Blynk
  Blynk.begin(auth, modem, apn, user, pass);
  
  // Setup timer for sending data
  timer.setInterval(60000L, sendSensorData); // Every minute
  
  Serial.println("Setup complete!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System ready");
}

void loop() {
  Blynk.run();
  timer.run();
} 