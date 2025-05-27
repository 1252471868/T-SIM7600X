# ESP32 to Arduino Mega Pump Controller - Wiring Guide

## Overview
This setup uses an ESP32 for Bluetooth communication and an Arduino Mega 2560 for actual pump control. The Arduino Mega provides the 5V capability needed for the air pumps.

## Connection Diagram

### ESP32 to Arduino Mega (Serial Communication)
```
ESP32 Pin 17 (TX2) -----> Arduino Mega Pin 0 (RX)
ESP32 Pin 16 (RX2) -----> Arduino Mega Pin 1 (TX)
ESP32 GND ---------------> Arduino Mega GND
```

### Arduino Mega to Pumps and Valve
```
Arduino Mega Pin 2 -----> Pump 1 ON/OFF Control (via relay/transistor)
Arduino Mega Pin 3 -----> Pump 2 ON/OFF Control (via relay/transistor)
Arduino Mega Pin 4 -----> Valve Control (via relay/transistor)
Arduino Mega Pin 5 -----> Pump 1 PWM Speed Control (via relay/transistor)
Arduino Mega Pin 6 -----> Pump 2 PWM Speed Control (via relay/transistor)
Arduino Mega 5V --------> Pump Power Supply (if direct connection)
Arduino Mega GND -------> Common Ground
```

## Hardware Requirements

### For Arduino Mega Pump Control:
1. **Relays or Transistors**: Since air pumps typically require more current than Arduino pins can provide
   - Option 1: 5V Relay modules (3 relays needed)
   - Option 2: MOSFET/Transistor circuits for switching

2. **External Power Supply**: If pumps require more than what Arduino can provide
   - 5V power supply for pumps
   - Connect ground to Arduino GND

### Recommended Relay Connection (if using relays):
```
Arduino Pin 2 -> Relay 1 Control -> Pump 1 Power
Arduino Pin 3 -> Relay 2 Control -> Pump 2 Power  
Arduino Pin 4 -> Relay 3 Control -> Valve Power
```

## Software Configuration

### ESP32 Configuration:
- Uses Serial2 (pins 16, 17) for Arduino communication
- Bluetooth for main ESP32 communication
- 9600 baud rate for Arduino communication
- 1-second timeout for Arduino responses

### Arduino Mega Configuration:
- Uses Serial (pins 0, 1) for ESP32 communication
- Digital pins 2, 3, 4 for pump/valve ON/OFF control
- PWM pins 5, 6 for pump speed control (duty cycle 0-255)
- 9600 baud rate
- JSON command processing (no response needed)
- Requires ArduinoJson library

## Command Flow
1. Main ESP32 -> ESP32 Pump Module (via Bluetooth JSON)
2. ESP32 Pump Module -> Arduino Mega (via Serial JSON commands with duty cycle)
3. Arduino Mega -> Physical Pumps/Valve (via digital + PWM pins)
4. ESP32 Pump Module -> Main ESP32 (JSON response)

## JSON Commands Supported

### Main ESP32 to ESP32 Pump Module (Bluetooth):
- `{"cmd":"P_INFLATE","duty_cycle":200}`: Inflate with duty cycle 200 (flow rate)
- `{"cmd":"P_DEFLATE","duty_cycle":150}`: Deflate with duty cycle 150
- `{"cmd":"P_STOP"}`: Stop all pumps

### ESP32 Pump Module to Arduino Mega (Serial):
- `{"cmd":"INFLATE","data":"200"}`: Turn on pumps at duty cycle 200
- `{"cmd":"DEFLATE","data":"150"}`: Turn on pumps at duty cycle 150
- `{"cmd":"STOP","data":""}`: Turn off all pumps and valve

### ESP32 Pump Module Response (Bluetooth):
- `{"cmd":"ACK","data":"P_INFLATE"}`: Command executed successfully
- `{"cmd":"NACK","data":"JSON_PARSE_ERROR"}`: Command failed

## Testing
1. Upload Arduino code to Mega 2560
2. Upload ESP32 code to ESP32
3. Connect serial pins between devices
4. Power both devices
5. Test with Bluetooth commands from main ESP32

## Troubleshooting
- Check serial connections if no response from Arduino
- Verify power supply can handle pump current requirements
- Use Serial Monitor on Arduino for debugging command reception
- Check ESP32 Serial Monitor for Arduino communication status 