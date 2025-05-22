#include "sensors.h"

// Globals are accessed via extern declarations in globals.h
// Sensor values are read directly via analogRead or potentially stored in globals later

/**
 * @brief Reads the voltage from the specified analog pin and calculates the battery voltage.
 * Assumes a voltage divider circuit is used.
 *
 * @param pin The analog pin connected to the battery voltage sense point.
 * @return float The calculated battery voltage.
 */
float readBattery(uint8_t pin) {
    int vref = 1100; // Internal reference voltage (may need calibration)
    uint16_t adcValue = analogRead(pin);
    // Calculation based on ESP32 ADC range (0-4095), 3.3V ref, 2x divider, and vref scaling
    // Adjust formula based on your specific voltage divider and ADC setup
    float battery_voltage = ((float)adcValue / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return battery_voltage;
} 