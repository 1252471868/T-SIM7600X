/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  Attention! Please check out TinyGSM guide:
    https://tiny.cc/tinygsm-readme

  Change GPRS apm, user, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!

 *************************************************************/

/* Fill-in your Template ID (only if using Blynk.Cloud) */
#define BLYNK_TEMPLATE_ID "TMPL6kFMi5YBK"
#define BLYNK_TEMPLATE_NAME "test"
#define BLYNK_AUTH_TOKEN "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI"

// Select your modem:
#define TINY_GSM_MODEM_SIM7600

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
//#define BLYNK_HEARTBEAT 30

#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

#include <SPI.h>
#include <SD.h>

Adafruit_BMP085 bmp;
BlynkTimer timer;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[]  = "";
char user[] = "";
char pass[] = "";

#define SerialAT Serial1
#define UART_BAUD           115200

#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34
#define BAT_ADC             35

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13

#define LED_PIN             12
bool reply = false;

// Add this structure after the existing includes
struct SensorData {
    String time;
    float temperature;
    float humidity;
    float pressure;
    float CO_W;
    float CO_A;
    float SO2_W;
    float SO2_A;
    float NO2_W;
    float NO2_A;
    float OX_W;
    float OX_A;
    float PID_W;
    float CO2_W;
};

// Add these global variables after other global declarations
File dataFile;
unsigned long lastFilePosition = 0;
String currentLine = "";

// Replace the readLatestSensorData function with these two functions
void initializeSDCardReader() {
    dataFile = SD.open("/WDATA2.TXT");
    if (!dataFile) {
        Serial.println("Failed to open WDATA2.TXT");
        return;
    }
    // Skip header line
    dataFile.readStringUntil('\n');
    lastFilePosition = dataFile.position();
}

SensorData readNextSensorData() {
    SensorData data;
    
    if (!dataFile) {
        Serial.println("File not opened");
        return data;
    }

    // Check if we've reached the end of file
    if (!dataFile.available()) {
        // Reset to beginning (after header)
        dataFile.seek(lastFilePosition);
        return data;
    }

    // Read next line
    String line = dataFile.readStringUntil('\n');
    lastFilePosition = dataFile.position();

    if (line.length() > 0) {
        int index = 0;
        int prevIndex = 0;
        
        // Parse time
        index = line.indexOf(' ');
        data.time = line.substring(0, index);
        prevIndex = index + 1;
        
        // Parse temperature
        index = line.indexOf(' ', prevIndex);
        data.temperature = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse humidity
        index = line.indexOf(' ', prevIndex);
        data.humidity = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse pressure
        index = line.indexOf(' ', prevIndex);
        data.pressure = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse CO_W
        index = line.indexOf(' ', prevIndex);
        data.CO_W = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse CO_A
        index = line.indexOf(' ', prevIndex);
        data.CO_A = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse SO2_W
        index = line.indexOf(' ', prevIndex);
        data.SO2_W = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse SO2_A
        index = line.indexOf(' ', prevIndex);
        data.SO2_A = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse NO2_W
        index = line.indexOf(' ', prevIndex);
        data.NO2_W = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse NO2_A
        index = line.indexOf(' ', prevIndex);
        data.NO2_A = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse OX_W
        index = line.indexOf(' ', prevIndex);
        data.OX_W = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse OX_A
        index = line.indexOf(' ', prevIndex);
        data.OX_A = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse PID_W
        index = line.indexOf(' ', prevIndex);
        data.PID_W = line.substring(prevIndex, index).toFloat();
        prevIndex = index + 1;
        
        // Parse CO2_W (last value)
        data.CO2_W = line.substring(prevIndex).toFloat();

        Serial.print("Read line: "); Serial.println(line);
    }

    return data;
}

BLYNK_WRITE(V3)
{
    if (param.asInt() == 1) {

        digitalWrite(LED_PIN, LOW);
        Blynk.logEvent("led_off");//Sending Events
    } else {
        digitalWrite(LED_PIN, HIGH);
        Blynk.logEvent("led_on");//Sending Events
    }
}

//Syncing the output state with the app at startup
BLYNK_CONNECTED()
{
    Blynk.syncVirtual(V3);  // will cause BLYNK_WRITE(V3) to be executed
}


float readBattery(uint8_t pin)
{
    int vref = 1100;
    uint16_t volt = analogRead(pin);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    return battery_voltage;
}

// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void sendSensor()
{
    SensorData sensorData = readNextSensorData();
    
    // float mv = readBattery(BAT_ADC);
    // Serial.print("Time: "); Serial.println(sensorData.time);
    // Serial.print("Temperature: "); Serial.println(sensorData.temperature);
    // Serial.print("Pressure: "); Serial.println(sensorData.pressure);
    // Serial.print("Battery: "); Serial.println(mv);
    
    // Send the actual sensor data to Blynk
    Blynk.virtualWrite(V0, sensorData.temperature);
    Blynk.virtualWrite(V1, sensorData.pressure);
    // Blynk.virtualWrite(V2, ((mv / 4200) * 100));
    // Add more virtual pins for other sensors
    Blynk.virtualWrite(V4, sensorData.humidity);
    Blynk.virtualWrite(V5, sensorData.CO_W);
    Blynk.virtualWrite(V6, sensorData.SO2_W);
    Blynk.virtualWrite(V7, sensorData.NO2_W);
    Blynk.virtualWrite(V8, sensorData.OX_W);
    Blynk.virtualWrite(V9, sensorData.PID_W);
    Blynk.virtualWrite(V10, sensorData.CO2_W);
}

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif


void setup()
{
    // Set console baud rate
    SerialMon.begin(115200);
    delay(10);

    // Set GSM module baud rate
    SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

    /*
      The indicator light of the board can be controlled
    */
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    /*
      MODEM_PWRKEY IO:4 The power-on signal of the modulator must be given to it,
      otherwise the modulator will not reply when the command is sent
    */
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(300); //Need delay
    digitalWrite(MODEM_PWRKEY, LOW);

    /*
      MODEM_FLIGHT IO:25 Modulator flight mode control,
      need to enable modulator, this pin must be set to high
    */
    pinMode(MODEM_FLIGHT, OUTPUT);
    digitalWrite(MODEM_FLIGHT, HIGH);

    //Initialize SDCard
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) { 
        Serial.println("SDCard MOUNT FAIL");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        String str = "SDCard Size: " + String(cardSize) + "MB";
        Serial.println(str);
        initializeSDCardReader();
    }



    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    Serial.println("Initializing modem...");
    if (!modem.restart()) {
        Serial.println("Failed to restart modem, attempting to continue without restarting");
    }

    String name = modem.getModemName();
    delay(500);
    Serial.println("Modem Name: " + name);


    // Launch BMP085
    // if (!bmp.begin()) {
    //     Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    //     while (1) {}
    // }



    Blynk.begin(auth, modem, apn, user, pass);
    // Change interval to 10 seconds (10000L milliseconds)
    timer.setInterval(10000L, sendSensor);
}

void loop()
{

    Blynk.run();
    timer.run();

}

// Add cleanup in case of reset or error
void cleanup() {
    if (dataFile) {
        dataFile.close();
    }
}
