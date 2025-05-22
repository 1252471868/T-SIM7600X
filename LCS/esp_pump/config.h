#ifndef ESP_PUMP_CONFIG_H
#define ESP_PUMP_CONFIG_H

// Bluetooth Configuration
#define PUMP_BT_DEVICE_NAME "ESP32_PumpModule"

// Pin Definitions
#define PUMP1_PIN 22 // GPIO for Pump 1 PWM
#define PUMP2_PIN 21 // GPIO for Pump 2 PWM
#define VALVE_PIN 23 // GPIO for Valve control

// PWM Configuration for Pumps
#define PWM_FREQ 5000     // PWM frequency in Hz
#define PWM_RESOLUTION 8  // PWM resolution (8-bit: 0-255)
#define PUMP_PWM_CHANNEL_1 0 // LEDC channel 0 for Pump 1
#define PUMP_PWM_CHANNEL_2 1 // LEDC channel 1 for Pump 2
#define PUMP_DUTY_CYCLE 200 // Default PWM duty cycle (0-255) for pumps when ON

// Operation Timing
#define PUMP_DURATION_MS 10000 // 10 seconds pump operation time

// Command Strings (to be received from main ESP32)
#define CMD_PUMP_INFLATE "P_INFLATE"
#define CMD_PUMP_DEFLATE "P_DEFLATE"
#define CMD_PUMP_STOP "P_STOP" // For immediate stop

// Response Command Strings
#define CMD_ACK "ACK"     // Acknowledgment response
#define CMD_NACK "NACK"   // Negative Acknowledgment response

#endif // ESP_PUMP_CONFIG_H 