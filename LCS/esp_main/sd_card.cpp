#include "global.h"
#include "sd_card.h"
#include "system_utils.h"

// Define global variables specific to SD Card module
File SDstorage; // File object for SD card logging
String filename = "/DAT"; // Base filename (will be updated in setupSD)

/**
 * @brief Creates a new data filename for the SD card.
 *
 * Generates a filename based on the current date and time in the format
 * "/YYYYMMDD_HHMMSS.txt" if the system time is valid (year > 2020).
 * If the time is not valid, it falls back to a numbered filename format
 * "/DATn.txt", incrementing 'n' until an unused filename is found.
 *
 * @return String The generated filename, including the leading slash.
 */
String createDataFilename() {
    char dateTimeFilename[30];
    bool timeValid = (year() > 2020); // Basic check if system time seems valid
    String newFilename;

    if (timeValid) {
        // Time is valid, use timestamp format
        sprintf(dateTimeFilename, "/%04d%02d%02d_%02d%02d%02d.txt", // Use underscore instead of space
                year(), month(), day(), hour(), minute(), second());
        newFilename = String(dateTimeFilename);
        Serial.println("Using timestamp filename: " + newFilename);
    } else {
        // Time not set, use fallback numbered format
        Serial.println("Time not set, using fallback filename format.");
        bool fileFound = false;
        int count = 0;
        String baseFilename = "/DAT";
        while (!fileFound) {
            String testFilename = baseFilename + String(count) + ".txt";
            // Need to ensure SD is initialized before calling exists
            // SD.begin() should be called in setupSD first.
            if (SD.exists(testFilename)) { // This assumes SD is mounted
                count++;
            } else {
                newFilename = testFilename;
                fileFound = true;
            }
        }
        Serial.println("Using fallback filename: " + newFilename);
    }

    filename = newFilename; // Update global filename variable
    return newFilename;
}

/**
 * @brief Initializes the SD card and creates the initial data log file.
 *
 * Sets up the SPI communication for the SD card, initializes the SD library,
 * creates the first data filename using createDataFilename(), and opens
 * the file, writing the header row.
 */
void setupSD() {
    Serial.println("Initializing SD card...");
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS); // Initialize SPI pins

    // Attempt to initialize SD card
    if (!SD.begin(SD_CS)) { 
        Serial.println("SD card initialization failed! Check connection and formatting.");
        // Consider halting setup or using an error LED pattern
        return; // Cannot proceed without SD card
    }

    Serial.println("SD card initialized successfully.");

    // Create the initial filename for logging
    createDataFilename(); // Create and update global filename

    // Open the initial file for writing (creates if not exists, truncates if exists)
    SDstorage = SD.open(filename.c_str(), FILE_WRITE);
    if (SDstorage) {
        Serial.println("Opened initial data file: " + filename);
        // Write the header row to the new file
        SDstorage.println("Time,Temperature(C),Humidity(%),Pressure(hPa),CO_W,CO_A,SO2_W,SO2_A,NO2_W,NO2_A,OX_W,OX_A,PID_W,CO2_W"); // Using CSV format
        SDstorage.flush(); // Ensure header is written immediately
        Serial.println("Wrote header to data file.");
    } else {
        Serial.println("Failed to open initial data file for writing! Check SD card.");
        // Consider halting setup or using an error LED pattern
    }
}

/**
 * @brief Attempts to reconnect to the SD card if initialization fails or connection is lost.
 * Loops indefinitely until SD.begin() succeeds.
 */
void reconnectSD() {
    Serial.println("Attempting to reconnect SD card...");
    while (!SD.begin(SD_CS)) {
        Serial.print("."); // Print dots while waiting
        delay(500);
        esp_task_wdt_reset(); // Reset watchdog while trying to reconnect
    }
    Serial.println("\nSD card reconnected successfully.");
}

/**
 * @brief Logs the current sensor data to the SD card.
 *
 * Handles SD card reconnection, file size checking (creates new file if > MAX_FILE_SIZE),
 * and writing the data row in CSV format. Includes watchdog timer resets.
 */
void logToSD() {
    // Reset watchdog timer before potentially long SD operations
    esp_task_wdt_reset();

    // 1. Ensure SDstorage object is valid and file is open
    // Check if the currently intended file exists. If not, SD might have been removed/reset.
    if (!SD.exists(filename.c_str())) {
        Serial.println("Current log file not found. Attempting SD reconnect/reopen...");
        reconnectSD(); // Try to re-initialize SD communication
        // Attempt to reopen the intended file in APPEND mode
        SDstorage = SD.open(filename.c_str(), FILE_APPEND);
        if (!SDstorage) {
            Serial.println("Failed to reopen file after reconnect! Trying to create new file...");
            // If reopening fails, try creating a completely new file
            createDataFilename();
            SDstorage = SD.open(filename.c_str(), FILE_WRITE);
            if(SDstorage) {
                SDstorage.println("Time,Temperature(C),Humidity(%),Pressure(hPa),CO_W,CO_A,SO2_W,SO2_A,NO2_W,NO2_A,OX_W,OX_A,PID_W,CO2_W");
                SDstorage.flush();
                Serial.println("Created new file after reopen failure: " + filename);
            } else {
                Serial.println("CRITICAL: Failed to open any file for logging!");
                return; // Cannot log
            }
        } else {
             Serial.println("Successfully reopened file: " + filename);
        }
    } else if (!SDstorage) {
        // If file exists but SDstorage is somehow invalid, try reopening it
        Serial.println("SDstorage object invalid, attempting to reopen file: " + filename);
        SDstorage = SD.open(filename.c_str(), FILE_APPEND);
        if (!SDstorage) {
            Serial.println("Failed to reopen existing file!");
            return; // Cannot log
        }
    }

    // 2. Proceed only if the file handle (SDstorage) is valid
    if (SDstorage) {
        unsigned long fileSize = SDstorage.size();
        // Serial.printf("Current file size: %lu bytes\n", fileSize); // Verbose logging

        // 3. Check if file size exceeds the limit
        if (fileSize > MAX_FILE_SIZE) {
            Serial.printf("File size limit (%lu bytes) reached. Creating new file.\n", MAX_FILE_SIZE);
            SDstorage.close(); // Close the current large file

            // Create a new filename
            createDataFilename(); // Create and update global filename

            // Open the new file (Write mode implicitly creates/truncates)
            SDstorage = SD.open(filename.c_str(), FILE_WRITE);
            if (SDstorage) {
                Serial.println("Created and opened new data file: " + filename);
                // Write the header to the new file
                SDstorage.println("Time,Temperature(C),Humidity(%),Pressure(hPa),CO_W,CO_A,SO2_W,SO2_A,NO2_W,NO2_A,OX_W,OX_A,PID_W,CO2_W");
                SDstorage.flush(); // Ensure header is written
            } else {
                Serial.println("CRITICAL: Failed to create new data file after size limit reached!");
                // Attempt to reopen the *old* file as a last resort? Or just fail?
                // For now, just return, logging will fail until next cycle/reconnect.
                return; 
            }
        }

        // 4. Format the timestamp for the log entry
        char timeStr[25];
        sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", // ISO-like format is better for logs
                year(), month(), day(), hour(), minute(), second());

        // 5. Record start time for write operation timing
        unsigned long writeStartTime = millis();

        // 6. Write the data row in CSV format
        SDstorage.print(timeStr);           SDstorage.print(",");
        SDstorage.print(temperature, 4);    SDstorage.print(",");
        SDstorage.print(humidity, 4);       SDstorage.print(",");
        SDstorage.print(pressure / 100.0, 4); SDstorage.print(","); // Convert Pa to hPa for header consistency
        SDstorage.print(v_CO_w, 4);         SDstorage.print(",");
        SDstorage.print(v_CO_a, 4);         SDstorage.print(",");
        SDstorage.print(v_SO2_w, 4);        SDstorage.print(",");
        SDstorage.print(v_SO2_a, 4);        SDstorage.print(",");
        SDstorage.print(v_NO2_w, 4);        SDstorage.print(",");
        SDstorage.print(v_NO2_a, 4);        SDstorage.print(",");
        SDstorage.print(v_OX_w, 4);         SDstorage.print(",");
        SDstorage.print(v_OX_a, 4);         SDstorage.print(",");
        SDstorage.print(v_pid_w, 4);        SDstorage.print(",");
        SDstorage.println(v_co2_w, 4);

        // 7. Flush data to SD card to ensure it's written
        SDstorage.flush();

        // 8. Check if the write operation took an unusually long time
        unsigned long writeDuration = millis() - writeStartTime;
        if (writeDuration > 5000) { // Threshold for warning (5 seconds)
            Serial.printf("WARNING: SD write/flush operation took %lu ms!\n", writeDuration);
        }

        // 9. Reset watchdog timer again after successful SD operations
        esp_task_wdt_reset();
    } else {
        // This should ideally not happen if file opening logic works
        Serial.println("SD file handle (SDstorage) is invalid, cannot log data!");
    }
} 