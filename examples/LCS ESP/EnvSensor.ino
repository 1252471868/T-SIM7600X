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

// Send sensor data to Blynk
void sendSensorData() {
  // Read all sensors
  readSensors();
  
  // Send to Blynk (V0-V2 as in example, adding more virtual pins for other sensors)
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, ((readBattery(BAT_ADC) / 4200) * 100)); // Battery percentage
  
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
  
  // Also log to SD if logging is enabled
  if (!stopReading) {
    logToSD();
  }
  
  // Print to serial for debug
  Serial.println("Data sent to Blynk");
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

// Read all sensors
void readSensors() {
  // BME680 sensor for temperature, humidity, pressure
  if (!bme.performReading()) {
    Serial.println("Failed to read BME680");
    return;
  }
  
  temperature = bme.temperature;
  humidity = bme.humidity;
  pressure = bme.pressure;
  
  // Alphasense gas sensors
  v_CO_w = analogRead(pin_CO_w) / 4095.0 * voltageSupply;
  v_CO_a = analogRead(pin_CO_a) / 4095.0 * voltageSupply;
  v_SO2_w = analogRead(pin_SO2_w) / 4095.0 * voltageSupply;
  v_SO2_a = analogRead(pin_SO2_a) / 4095.0 * voltageSupply;
  v_NO2_w = analogRead(pin_NO2_w) / 4095.0 * voltageSupply;
  v_NO2_a = analogRead(pin_NO2_a) / 4095.0 * voltageSupply;
  v_OX_w = analogRead(pin_OX_w) / 4095.0 * voltageSupply;
  v_OX_a = analogRead(pin_OX_a) / 4095.0 * voltageSupply;
  v_pid_w = analogRead(pin_pid) / 4095.0 * voltageSupply;
  v_co2_w = analogRead(pin_CO2) / 4095.0 * voltageSupply;
  
  // Update LCD display
  lcd.clear();
  lcd.setCursor(0, 0);
  char timeBuffer[20];
  sprintf(timeBuffer, "%02d:%02d:%02d", hour(), minute(), second());
  lcd.print(timeBuffer);
  lcd.print(' ');
  lcd.print(int(temperature));
  lcd.print('C');
  lcd.print(' ');
  lcd.print(int(humidity));
  lcd.print('%');
  
  // Display gas sensor data
  lcd.setCursor(0, 1);
  lcd.print("CO:");
  lcd.print(int(v_CO_w * 100));
  lcd.print(" NO2:");
  lcd.print(int(v_NO2_w * 100));
}

// Log data to SD card
void logToSD() {
  if (!SD.begin(SD_CS)) {
    reconnectSD();
    return;
  }
  
  SDstorage = SD.open(filename, FILE_WRITE);
  if (SDstorage) {
    char timeBuffer[20];
    sprintf(timeBuffer, "%02d:%02d:%02d", hour(), minute(), second());
    
    SDstorage.print(timeBuffer);
    SDstorage.print(" ");
    SDstorage.print(temperature);
    SDstorage.print(" ");
    SDstorage.print(humidity);
    SDstorage.print(" ");
    SDstorage.print(pressure);
    SDstorage.print(" ");
    
    // Gas sensors
    SDstorage.print(v_CO_w, 3);
    SDstorage.print(" ");
    SDstorage.print(v_CO_a, 3);
    SDstorage.print(" ");
    SDstorage.print(v_SO2_w, 3);
    SDstorage.print(" ");
    SDstorage.print(v_SO2_a, 3);
    SDstorage.print(" ");
    SDstorage.print(v_NO2_w, 3);
    SDstorage.print(" ");
    SDstorage.print(v_NO2_a, 3);
    SDstorage.print(" ");
    SDstorage.print(v_OX_w, 3);
    SDstorage.print(" ");
    SDstorage.print(v_OX_a, 3);
    SDstorage.print(" ");
    SDstorage.print(v_pid_w, 3);
    SDstorage.print(" ");
    SDstorage.print(v_co2_w, 3);
    SDstorage.println("");
    
    SDstorage.flush();
    
    // Check if we need to create a new file due to size limit
    if (SDstorage.size() > MAX_FILE_SIZE) {
      SDstorage.close();
      
      // Create a new file with incremented name
      int suffix = 1;
      String newName;
      do {
        newName = filename.substring(0, filename.length() - 4) + "_" + String(suffix) + ".txt";
        suffix++;
      } while (SD.exists(newName));
      
      filename = newName;
      Serial.println("Created new log file: " + filename);
      
      // Write header to new file
      SDstorage = SD.open(filename, FILE_WRITE);
      if (SDstorage) {
        SDstorage.println("Time Temperature(C) Humidity(%) Pressure(pa) CO_W CO_A SO2_W SO2_A NO2_W NO2_A OX_W OX_A PID_W CO2_W");
        SDstorage.flush();
        SDstorage.close();
      }
    } else {
      SDstorage.close();
    }
  } else {
    Serial.println("Error opening SD card file for writing");
  }
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
  
  // Initialize BME680 sensor
  if (!bme.begin()) {
    Serial.println("Could not find BME680 sensor!");
    lcd.clear();
    lcd.print("BME680 error!");
    while (1) delay(10);
  }
  
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  
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