/**
 * @file      GPSDebug.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-06-13
 * @note      The diagram shows the result through external analysis analysis NMAE example,
 *            can be used for GPS failure judgment, can be used to see GPS correct or not operation normal.
 *            If the GPS output is NMEA phrase, it is possible to confirm that GPS is normal. https://github.com/Xinyuan-LilyGO/T-SIM7600X/issues/42#issuecomment-1507181275
 *            It's been a long time since I've been unlocated.
 * */
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

#define BOX_NUM 3
#define BOX_NAME "box3"
#define PUMP_NUM 1 // Which pump ESP32 to send VOC data to

// Blynk configuration
#define BLYNK_TEMPLATE_ID "TMPL6kFMi5YBK"
#define BLYNK_TEMPLATE_NAME "EnvSensor"
#define BLYNK_DOMAIN "sgp1.blynk.cloud"
#define BLYNK_PORT 8080
#if BOX_NUM == 1
#define BLYNK_AUTH_TOKEN "iihKlmC4B_tYYOZZS68Fm9H8PUJX7Ed_" // Replace with your token
#elif BOX_NUM == 2
#define BLYNK_AUTH_TOKEN "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI" // Replace with your token
#elif BOX_NUM == 3
#define BLYNK_AUTH_TOKEN "tzqMA1jqbtyY2iCwSWi6u34KtkcQKZ0L" // Replace with your token
#elif BOX_NUM == 4
#define BLYNK_AUTH_TOKEN "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI" // Replace with your token

#endif

// Use TinyGPS NMEA math analysis library
#define USING_TINYGPS_LIBRARY           https://github.com/mikalhart/TinyGPSPlus.git

// set GSM PIN, if any
#define GSM_PIN ""

#include <TinyGsmClient.h>
#include "utilities.h"
#include <BlynkSimpleTinyGSM.h>

#ifdef USING_TINYGPS_LIBRARY
// Use TinyGPS NMEA math analysis library
#include <TinyGPS++.h>
TinyGPSPlus gps;
void displayInfo();
#endif


#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#include <SPI.h>
#include <SD.h>
File dataFile;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[] = "";
char user[] = "";
char pass[] = "";
BlynkTimer timer;
#define VPIN_LOCATION V19           // Location
void setup()
{
    Serial.begin(115200); // Set console baud rate
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS))
    {
        Serial.println("SDCard MOUNT FAIL");
    }
    else
    {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        String str = "SDCard Size: " + String(cardSize) + "MB";
        Serial.println(str);
    }
    String filename = "/gps_raw.txt";
    int count = 0;
    while(SD.exists(filename))
    {
        filename = "/gps_raw_" + String(count) + ".txt";
        count++;
    }
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        Serial.println("Recording raw GPS output to " + filename);
    } else {
        Serial.println("Error opening " + filename);
    }
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

    Serial.println("Start modem...");


    for (int i = 0; i < 3; ++i) {
        while (!modem.testAT(5000)) {
            Serial.println("Try to start modem...");
            pinMode(MODEM_PWRKEY, OUTPUT);
            digitalWrite(MODEM_PWRKEY, HIGH);
            delay(300); //Need delay
            digitalWrite(MODEM_PWRKEY, LOW);
        }
    }
    Blynk.config(modem, auth, "sgp1.blynk.cloud");
    Blynk.connectNetwork(apn, user, pass);
    Blynk.connect(BLYNK_TIMEOUT_MS);
    // Blynk.begin(auth, modem, apn, user, pass);
    // Change interval to 10 seconds (10000L milliseconds)
    timer.setInterval(10000L, sendSensor);


    Serial.println("Modem Response Started.");
    Serial.println("Enabling GPS/GNSS/GLONASS");
    modem.enableGPS();
    delay(2000);

    // light_sleep(2);
    // Stop GPS Server
    // modem.sendAT("+CGPS=0");
    // modem.waitResponse(30000);

    // // Configure GNSS support mode
    // modem.sendAT("+CGNSSMODE=15,1");
    // modem.waitResponse(30000);

    // // Configure NMEA sentence type
    // modem.sendAT("+CGPSNMEA=200191");
    // modem.waitResponse(30000);

    // // Set NMEA output rate to 1HZ
    // modem.sendAT("+CGPSNMEARATE=1");
    // modem.waitResponse(30000);

    // // Enable GPS
    // modem.sendAT("+CGPS=1");
    // modem.waitResponse(30000);

    // // Download Report GPS NMEA-0183 sentence , NMEA TO AT PORT
    // modem.sendAT("+CGPSINFOCFG=1,31");
    // modem.waitResponse(30000);


    //Disable NMEA OUTPUT
    // modem.sendAT("+CGPSINFOCFG=0,31");
    // modem.waitResponse(30000);
}

float lat2 = 0;
float lon2 = 0;
float speed2 = 0;
float alt2 = 0;
int vsat2 = 0;
int usat2 = 0;
float accuracy2 = 0;
int year2 = 0;
int month2 = 0;
int day2 = 0;
int hour2 = 0;
int min2 = 0;
int sec2 = 0;
int loop_count = 0;

void sendSensor()
{
    Serial.println("Sending GPS data to Blynk");
    if (modem.getGPS(&lat2, &lon2))
    {
        Serial.println("Latitude: " + String(lat2, 8) + "\tLongitude: " + String(lon2, 8));
        Blynk.virtualWrite(VPIN_LOCATION, double(lat2), double(lon2));
        if(loop_count == 0)
        {
            dataFile.println("Latitude,Longitude");
        }
        dataFile.print(lat2, 8);
        dataFile.print(",");
        dataFile.print(lon2, 8);
        dataFile.println();
        dataFile.flush();
    }
    else
    {
        Serial.println("Invalid GPS data received");
    }
    loop_count++;
}


void loop()
{
    Blynk.run();
    timer.run();
    // Serial.println("Requesting current GPS/GNSS/GLONASS location");
    // if (loop_count == 0) {
    //     dataFile.println("Latitude,Longitude");
    // }
    // if (modem.getGPS(&lat2, &lon2))
    // {
    //     Serial.println("Latitude: " + String(lat2, 8) + "\tLongitude: " + String(lon2, 8));
    //     Serial.println("Speed: " + String(speed2) + "\tAltitude: " + String(alt2));
    //     Serial.println("Visible Satellites: " + String(vsat2) + "\tUsed Satellites: " + String(usat2));
    //     Serial.println("Accuracy: " + String(accuracy2));
    //     Serial.println("Year: " + String(year2) + "\tMonth: " + String(month2) + "\tDay: " + String(day2));
    //     Serial.println("Hour: " + String(hour2) + "\tMinute: " + String(min2) + "\tSecond: " + String(sec2));
    //     // dataFile.print("modem reading:");
    //     // dataFile.print("Latitude:");
    //     dataFile.print(lat2, 8);
    //     dataFile.print(',');
    //     // dataFile.print("Longitude:");
    //     dataFile.print(lon2, 8);
    //     dataFile.println();
    //     dataFile.flush();
    // }
    // else
    // {
    //     // light_sleep(2);
    //     Serial.println("Invalid GPS data received");
    //     delay(500);
    // }
    // // Serial.println("Retrieving GPS/GNSS/GLONASS location again as a string");
    // // String gps_raw = modem.getGPSraw();
    // // Serial.println("GPS/GNSS Based Location String: " + gps_raw);
    // // delay(1000);

    // // #ifdef USING_TINYGPS_LIBRARY
    // //     while (SerialAT.available()) {
    // //         if (gps.encode(SerialAT.read())) {
    // //             displayInfo();
    // //         }
    // //     }
    // // #else
    // //     if (SerialAT.available()) {
    // //         Serial.write(SerialAT.read());
    // //     }
    // //     if (Serial.available()) {
    // //         SerialAT.write(Serial.read());
    // //     }
    // // #endif
    //     delay(1000);
    //     loop_count++;
}


#ifdef USING_TINYGPS_LIBRARY
void displayInfo()
{
    Serial.print(F("Location: "));
    dataFile.print(F("Location: "));
    if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
        dataFile.print(gps.location.lat(), 6);
        dataFile.print(F(","));
        dataFile.print(gps.location.lng(), 6);
    } else {
        Serial.print(F("INVALID"));
        dataFile.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    dataFile.print(F("  Date/Time: "));
    if (gps.date.isValid()) {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
        dataFile.print(gps.date.month());
        dataFile.print(F("/"));
        dataFile.print(gps.date.day());
        dataFile.print(F("/"));
        dataFile.print(gps.date.year());
    } else {
        Serial.print(F("INVALID"));
        dataFile.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid()) {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
        dataFile.print(gps.time.hour());
        dataFile.print(F(":"));
        dataFile.print(gps.time.minute());
        dataFile.print(F(":"));
        dataFile.print(gps.time.second());
        dataFile.print(F("."));
        dataFile.print(gps.time.centisecond());
    } else {
        Serial.print(F("INVALID"));
        dataFile.print(F("INVALID"));
    }

    Serial.println();
    dataFile.println();
    dataFile.flush();
}
#endif