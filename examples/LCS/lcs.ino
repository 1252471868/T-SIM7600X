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
#include <SPI.h>
#include <SD.h>
#include <Time.h>
#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

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
// Sampling duration
/*
 * ************************************************************
 */
const long minute2run = 15;       // minutes in each loop
const long numberOfLoop = 100000; // number of loops to run, so total minutes to run is minute2run * numberOfLoop
// Define the maximum file size (in bytes)
const unsigned long MAX_FILE_SIZE = 1000000; // 1 MB

String filename = "DAT";

volatile bool stopReading = false;

const int buttonPin = 2;
const unsigned long debounceDelay = 50;
volatile unsigned long lastInterruptTime = 0;

void stopSDReading()
{
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > debounceDelay)
    {
        stopReading = true;
        lastInterruptTime = interruptTime;
        Serial.println("SD card reading stopped.");
        // SDstorage.close();
    }
}

// void beginSDReading() {
//   unsigned long interruptTime = millis();
//   if (interruptTime - lastInterruptTime > debounceDelay) {
//     stopReading = false;
//     lastInterruptTime = interruptTime;

//   }
// }

void SD_reconnect()
{
    while (!SD.begin())
    {
        lcd.setCursor(0, 0); // 2nd row, 1st col
        lcd.print("SD reconnecting");
        delay(500);
    }
    lcd.print("SD reconnected");
}

void setup()
{
    // begin serial communication
    Serial.begin(115200);
    delay(500);
    Serial.println("Initializing");
    attachInterrupt(digitalPinToInterrupt(buttonPin), stopSDReading, FALLING);
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

    // Serial.begin(9600);
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

    // initialize SD module
    if (!SD.begin())
    {
        while (1)
            ;
        delay(500);
    }
    Serial.println("2");

    Serial.println("SD card module initialized");
    delay(500);
    lcd.clear();
    lcd.setCursor(0, 0); // 2nd row, 1st col
    lcd.print("SD ready");
    delay(500);

    bool file = false; // tracks when we found a file name we can use
    int count = 0;     // used for naming the file
    while (!file)
    {
        if (SD.exists(filename + ".txt"))
        {
            count++;                          // up the count for naming the file
            filename = "DAT" + (String)count; // make a new name
        }
        else
            file = true; // we found a new name so we can exit loop
    }

    filename = filename + ".txt";
    setTime(0, 0, 0, 1, 9, 2022); // change it to the date of experiment
    // Oct 20, 2021 -> setTime(0, 0, 0, 20, 10, 2021);
    // Sept 9, 2021 -> setTime(0, 0, 0, 9, 9, 2021);

    lcd.setCursor(0, 1); // 2nd row, 1st col
    lcd.print(filename);
    Serial.print(filename);
    delay(3000);
    // starting measurement loop
    int loopNow = 0; // the current loop
    File SDstorage;
    SDstorage = SD.open(filename, FILE_WRITE);
    SDstorage.print("Time");
    SDstorage.print(" ");
    SDstorage.print("Temperature(C)");
    SDstorage.print(" ");
    SDstorage.print("Humidity(%)");
    SDstorage.print(" ");
    SDstorage.print("Pressure(pa)");
    SDstorage.print(" ");
    SDstorage.print("CO_W");
    SDstorage.print(" "); //~V, CO-B4, working voltage
    SDstorage.print("CO_A");
    SDstorage.print(" "); //~V, CO_B4, aux voltage
    SDstorage.print("SO2_W");
    SDstorage.print(" "); //~V, SO2-B4, working voltage
    SDstorage.print("SO2_A");
    SDstorage.print(" "); //~V, SO2_B4, aux voltage
    SDstorage.print("NO2_W");
    SDstorage.print(" "); //~V, NO2-B43F, working voltage
    SDstorage.print("NO2_A");
    SDstorage.print(" "); //~V, NO2-B43F, aux voltage
    SDstorage.print("OX_W");
    SDstorage.print(" "); //~V, OX-B431, working voltage
    SDstorage.print("OX_A");
    SDstorage.print(" "); //~V, OX-B431, aux voltage
    SDstorage.print("PID_W");
    SDstorage.print(" "); //~V, PID, working voltage
    SDstorage.print("CO2_W");
    SDstorage.print(" "); //~V, CO2, working voltage
    SDstorage.println(" ");
    lcd.print("File created");
    lcd.setCursor(0, 1); // 2nd line, 1st row
    SDstorage.flush();   // close SDstorage instance to save headings
    delay(1000);

    while (loopNow < numberOfLoop)
    {
        loopNow++;
        time_t startTimeOfThisLoop = now(); // starting time of this loop in seconds

        if (stopReading == true)
        {
            SDstorage.close();
            Serial.println("SD card reading stopped.");
        }

        if (SDstorage)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            Serial.print("Time");
            Serial.print(" ");
            Serial.print("Temperature(C)");
            Serial.print(" ");
            Serial.print("Humidity(%)");
            Serial.print(" ");
            Serial.print("Pressure(pa)");
            Serial.print(" ");
            Serial.print("CO_W");
            Serial.print(" "); //~V, CO-B4, working voltage
            Serial.print("CO_A");
            Serial.print(" "); //~V, CO_B4, aux voltage
            Serial.print("SO2_W");
            Serial.print(" "); //~V, SO2-B4, working voltage
            Serial.print("SO2_A");
            Serial.print(" "); //~V, SO2_B4, aux voltage
            Serial.print("NO2_W");
            Serial.print(" "); //~V, NO2-B43F, working voltage
            Serial.print("NO2_A");
            Serial.print(" "); //~V, NO2-B43F, aux voltage
            Serial.print("OX_W");
            Serial.print(" "); //~V, OX-B431, working voltage
            Serial.print("OX_A");
            Serial.print(" "); //~V, OX-B431, aux voltage
            Serial.print("PID_W");
            Serial.print(" "); //~V, PID, working voltage
            Serial.print("CO2_W");
            Serial.print(" "); //~V, CO2, working voltage
            Serial.println(" ");

            // record sensor readings and save onto SD card
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

            char timebuffer[10];
            char number2display[50]; // number to display (10 * original voltage) on LCD monitor

            time_t timeDuringRecording = now();
            if (stopReading == false)
            {
                // SDstorage = SD.open(filename, FILE_WRITE);
                do
                {
                    timeDuringRecording = now(); // time during recording loop, in loops

                    /*
                      Alphasense Sensors
                    */

                    double v_CO_w = analogRead(pin_CO_w) / 1024.000 * voltageSupply;
                    double v_CO_a = analogRead(pin_CO_a) / 1024.000 * voltageSupply;
                    double v_SO2_w = analogRead(pin_SO2_w) / 1024.000 * voltageSupply;
                    double v_SO2_a = analogRead(pin_SO2_a) / 1024.000 * voltageSupply;
                    double v_NO2_w = analogRead(pin_NO2_w) / 1024.000 * voltageSupply;
                    double v_NO2_a = analogRead(pin_NO2_a) / 1024.000 * voltageSupply;
                    double v_OX_w = analogRead(pin_OX_w) / 1024.000 * voltageSupply;
                    double v_OX_a = analogRead(pin_OX_a) / 1024.000 * voltageSupply;
                    double v_pid_w = analogRead(pin_pid) / 1024.000 * voltageSupply;
                    double v_co2_w = analogRead(pin_CO2) / 1024.000 * voltageSupply;

                    /*
                      Temperature and humidity sensor: BME 680
                    */
                    if (!bme.performReading())
                    {
                        Serial.println("Failed to perform reading :(");
                        return;
                    }
                    float rh = bme.humidity;   // relative humidity (%)
                    float t = bme.temperature; // temperature (C)
                    float p = bme.pressure;    // pressure (pa)

                    /*
                      LCD screen: v 4.0
                    */

                    long counter = 0;
                    while (counter < 2)
                    {
                        lcd.clear();

                        lcd.setCursor(0, 0);
                        sprintf(timebuffer, "%02d:%02d:%02d", hour(), minute(), second());
                        lcd.print(timebuffer);
                        lcd.print(' ');
                        lcd.print(int(t));
                        lcd.print(' ');
                        lcd.print(int(rh));
                        lcd.print(' ');

                        if (counter == 0)
                        {
                            lcd.print('W'); // indicate it is for working voltage
                            sprintf(number2display, "%02d %02d %02d %02d %03d", int(v_CO_w * 10), int(v_SO2_w * 10), int(v_NO2_w * 10), int(v_OX_w * 10), int(v_pid_w * 100));
                        }
                        else
                        {
                            lcd.print('A'); // indicate it is for auxiliary voltage
                            sprintf(number2display, "%02d %02d %02d %02d %03d", int(v_CO_a * 10), int(v_SO2_a * 10), int(v_NO2_a * 10), int(v_OX_a * 10), int(v_co2_w * 100));
                        }
                        lcd.setCursor(0, 1); // 2nd line, 1st row
                        lcd.print(number2display);

                        delay(1000);
                        counter++;
                    }

                    /*
                      SD module: write data
                    */

                    // sprintf(timebuffer, "%d:%d:%d", hour(), minute(), second());
                    if (SD.begin())
                    {
                        SDstorage.print(timebuffer);
                        SDstorage.print(" ");
                        SDstorage.print(bme.temperature);
                        SDstorage.print(" ");
                        SDstorage.print(bme.humidity);
                        SDstorage.print(" ");
                        SDstorage.print(bme.pressure);
                        SDstorage.print(" ");
                        SDstorage.print(v_CO_w, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_CO_a, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_SO2_w, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_SO2_a, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_NO2_w, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_NO2_a, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_OX_w, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_OX_a, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_pid_w, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.print(v_co2_w, 3);
                        SDstorage.print(" "); //~V
                        SDstorage.println(" ");
                    }
                    else
                    {
                        SD_reconnect();
                        SDstorage = SD.open(filename, FILE_WRITE);
                    }

                    /*
                      Serial communication
                    */

                    Serial.print(timebuffer);
                    Serial.print(" ");
                    Serial.print(t);
                    Serial.print(" ");
                    Serial.print(rh);
                    Serial.print(" ");
                    Serial.print(p);
                    Serial.print(" ");
                    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
                    Serial.print(v_CO_w, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_CO_a, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_SO2_w, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_SO2_a, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_NO2_w, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_NO2_a, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_OX_w, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_OX_a, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_pid_w, 3);
                    Serial.print(" "); //~V
                    Serial.print(v_co2_w, 3);
                    Serial.print(" "); //~V
                    Serial.println(" ");

                    delay(2000);
                    /*
                        if (hour() * 60 + minute() > previous_hour * 60 + previous_minute) {
                          lcd.clear();

                          char timecaller1[10], timecaller2[10];
                          sprintf(timecaller1, "Minute %d", hour() * 60 + minute());
                          sprintf(timecaller2, "%d left", numberOfLoop * minute2run - hour() * 60 - minute());
                          lcd.setCursor(0, 0); lcd.print(timecaller1);
                          lcd.setCursor(0, 1); lcd.print(timecaller2);
                          delay(1000);
                        }
                    */
                } while (timeDuringRecording - startTimeOfThisLoop < minute2run * 60 && !stopReading); // if time elapsed does not exceed specified duration

                SDstorage.flush();

                // Check if the current file size exceeds our threshold.
                if (SDstorage.size() > MAX_FILE_SIZE)
                {
                    SDstorage.close();
                    delay(500);
                    if (SDstorage)
                    {
                        Serial.print("321\n");
                    }
                    else
                    {
                        // Build a new name by appending an incremental suffix.
                        // For example, if filename is "WDATA1.txt", we try "WDATA1_1.txt", "WDATA1_2.txt", etc.
                        int suffix = 1;
                        String newName;
                        do
                        {
                            // Remove the ".txt" extension and add "_" plus the suffix and then ".txt"
                            newName = filename.substring(0, filename.length() - 4) + "_" + String(suffix) + ".txt";
                            suffix++;
                        } while (SD.exists(newName));

                        // Rename the current file.
                        // filename = newName;
                        Serial.print(newName);
                        Serial.print(filename);

                        // Reopen a fresh file with the original filename for new data.
                        SDstorage = SD.open(newName, FILE_WRITE);
                        delay(500);
                        if (!SDstorage)
                            Serial.print("failed");
                    }
                }
            }
        }
        else
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            if (stopReading)
            {
                lcd.print("SD card stopped");
                Serial.println("SD card reading stopped");
            }
            else
            {
                lcd.print("SD module fails");
                Serial.println("SD module fails");
                SD_reconnect();
            }

            // RED for error
            int colorR = 236;
            int colorG = 0;
            int colorB = 0;

            lcd.setRGB(colorR, colorG, colorB);
            delay(2000);
        }
    }
}

void loop() {}
