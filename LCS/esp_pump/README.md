# ESP32 Pump Controller with Blynk Integration

## Overview

This ESP32 pump controller receives commands directly from a Blynk dashboard via cellular internet connection (SIM7600X modem) and controls air pumps through an Arduino Mega 2560. The system supports multiple operation modes including manual control, VOC-triggered automation, and timed sampling.

## Architecture

```
Main ESP32 (Air Quality) ←→ ESP32 Pump Controller (SIM7600X) ←→ Arduino Mega 2560 → Air Pumps
                Bluetooth                    Blynk Dashboard
```

- **Main ESP32**: Air quality monitoring system with VOC sensors
- **ESP32 Pump Controller**: Handles Blynk communication, internet connectivity, and high-level pump logic
- **Arduino Mega**: Controls physical pump hardware (5V requirement)
- **SIM7600X Modem**: Provides cellular internet connectivity
- **Bluetooth**: Communication between main ESP32 and pump controller for VOC data
- **Air Pumps**: Require 5V control signals (hence Arduino Mega requirement)

## Features

### 1. Manual Pump Control
- **Virtual Pin V0**: Pump ON/OFF switch
- **Virtual Pin V1**: Flow rate control (0-255 PWM duty cycle)
- Real-time pump status display
- Safety timeout (5 minutes max runtime)

### 2. Flow Rate Control
- Adjustable pump speed via PWM (0-255)
- Real-time flow rate updates while pump is running
- Validation and safety limits

### 3. VOC Auto-Trigger
- **Virtual Pin V2**: Enable/disable VOC auto-trigger
- **Virtual Pin V3**: VOC concentration threshold setting
- **Virtual Pin V8**: Current VOC level display (read-only)
- **Virtual Pin V11**: Main ESP32 connection status (read-only)
- Automatic sampling when threshold is exceeded
- Configurable threshold limits (0-1000)
- Real-time VOC data from main ESP32 via Bluetooth

### 4. VOC Concentration Threshold
- User-configurable trigger level
- Real-time monitoring and display
- Event logging when threshold is exceeded
- Automatic pump activation for sampling

### 5. Timed Start
- **Virtual Pin V4**: Enable/disable timed operation
- **Virtual Pin V5**: Schedule start time (minutes from now)
- **Virtual Pin V6**: Sampling duration (seconds)
- Flexible scheduling (up to 24 hours in advance)
- Automatic execution at scheduled time

### 6. Sampling Time Control
- Configurable sampling duration (1-3600 seconds)
- Automatic pump stop after sampling period
- Progress tracking and status updates
- Integration with both VOC auto-trigger and timed modes

## File Structure

```
LCS/esp_pump/
├── main.ino              # Main program entry point
├── global.h              # Global constants and variable declarations
├── global.cpp            # Global variable definitions
├── blynk_interface.h     # Blynk communication header
├── blynk_interface.cpp   # Blynk virtual pin handlers and connectivity
├── pump_control.h        # Pump control functions header
├── pump_control.cpp      # Arduino communication and pump logic
├── bluetooth_comm.h      # Bluetooth communication header
├── bluetooth_comm.cpp    # Bluetooth communication with main ESP32
├── modem_setup.h         # Modem initialization header
├── modem_setup.cpp       # SIM7600X modem setup and time sync
├── system_utils.h        # System utilities header
├── system_utils.cpp      # Watchdog and system reset functions
└── README.md             # This file
```

## Virtual Pin Mapping

| Pin | Function | Type | Range | Description |
|-----|----------|------|-------|-------------|
| V0  | Pump Enable | Switch | 0/1 | Manual pump ON/OFF |
| V1  | Flow Rate | Slider | 0-255 | PWM duty cycle for pump speed |
| V2  | VOC Auto Enable | Switch | 0/1 | Enable VOC auto-trigger |
| V3  | VOC Threshold | Number | 0-1000 | VOC concentration trigger level |
| V4  | Timed Start Enable | Switch | 0/1 | Enable timed operation |
| V5  | Timed Start Time | Number | 0-1440 | Minutes from now to start |
| V6  | Sampling Time | Number | 1-3600 | Sampling duration in seconds |
| V7  | Pump Status | Display | Text | Current pump status (read-only) |
| V8  | Current VOC | Display | Number | Current VOC reading (read-only) |
| V9  | Pump Mode | Display | Number | Current operation mode (read-only) |
| V10 | Arduino Status | LED | 0/1 | Arduino connection status |
| V11 | Main ESP32 Status | LED | 0/1 | Main ESP32 Bluetooth connection status |
| V12 | System Reset | Button | - | Restart ESP32 system |
| V13 | Emergency Stop | Button | - | Immediate stop all operations |

## Operation Modes

### PUMP_OFF (0)
- All pump operations disabled
- System in standby mode

### PUMP_MANUAL (1)
- Direct user control via Blynk dashboard
- Manual flow rate adjustment
- Safety timeout protection

### PUMP_VOC_AUTO (2)
- Automatic triggering based on VOC levels
- Continuous monitoring mode
- Threshold-based activation

### PUMP_TIMED (3)
- Scheduled operation mode
- User-defined start time and duration
- One-time or recurring schedules

### PUMP_SAMPLING (4)
- Active sampling mode
- Fixed duration operation
- Triggered by VOC or timer

## Hardware Connections

### ESP32 to Arduino Mega
- ESP32 Pin 16 (RX) → Arduino Pin 1 (TX)
- ESP32 Pin 17 (TX) → Arduino Pin 0 (RX)
- Common Ground

### ESP32 to SIM7600X Modem
- ESP32 Pin 26 (RX) → SIM7600X TX
- ESP32 Pin 27 (TX) → SIM7600X RX
- Power and control pins as defined in global.h

### ESP32 to Main ESP32 (Bluetooth)
- Bluetooth Classic pairing between devices
- ESP32 Pump Controller: "ESP32_PumpController_BT"
- Main ESP32: "ESP32_EnvSensor_BT"

### Arduino Mega to Pumps
- Pin 2: Pump 1 ON/OFF
- Pin 3: Pump 2 ON/OFF
- Pin 4: Valve control
- Pin 5: Pump 1 PWM speed
- Pin 6: Pump 2 PWM speed

## Communication Protocol

### ESP32 → Arduino Commands
```json
{"cmd":"INFLATE","data":"255"}
{"cmd":"DEFLATE","data":"128"}
{"cmd":"STOP","data":""}
{"cmd":"INFO","data":""}
```

### ESP32 ↔ Main ESP32 (Bluetooth)
```json
// Request VOC data
{"cmd":"DATA","data":""}

// Response with sensor data (VOC at index 13)
{"cmd":"DATA","data":[timestamp,temp,humidity,pressure,battery,co_w,co_a,so2_w,so2_a,no2_w,no2_a,ox_w,ox_a,voc_w,co2_w]}

// Acknowledgment
{"cmd":"ACK","data":""}
```

### Command Types
- **INFLATE**: Start inflation with specified duty cycle
- **DEFLATE**: Start deflation with specified duty cycle
- **STOP**: Stop all pump operations
- **INFO**: Check Arduino status
- **DATA**: Request/send sensor data (Bluetooth)
- **ACK**: Acknowledge received command

## Configuration

### Blynk Setup
1. Replace `YOUR_BLYNK_AUTH_TOKEN` in `blynk_interface.cpp`
2. Configure APN settings for your cellular carrier
3. Set up virtual pins in Blynk dashboard according to mapping table

### Cellular Configuration
- Update APN, username, password in `blynk_interface.cpp`
- Verify SIM card is activated and has data plan
- Check signal strength and network coverage

### Bluetooth Configuration
- Ensure main ESP32 has `ENABLE_PUMP_CONTROL` enabled
- Pair ESP32 devices using Bluetooth Classic
- Verify device names match in both systems
- Check Bluetooth connection status via V11 virtual pin

### Safety Settings
- Modify timeout values in `global.h` if needed
- Adjust maximum runtime limits in `pump_control.cpp`
- Configure watchdog timeout for system stability

## Safety Features

### Automatic Timeouts
- Manual mode: 5-minute maximum runtime
- Communication timeout: Arduino connection monitoring
- Watchdog timer: System reset if frozen

### Emergency Controls
- Emergency stop button (V12) - immediate halt
- System reset button (V11) - restart ESP32
- Automatic pump stop on communication loss

### Validation
- Flow rate limits (0-255)
- Threshold validation (0-1000)
- Time range validation (1-3600 seconds)

## Monitoring and Logging

### Status Updates
- Real-time pump status display
- Arduino connection monitoring
- VOC level tracking
- Operation mode indication

### Event Logging
- Blynk event notifications for key actions
- Serial console debugging output
- Error condition reporting

## Troubleshooting

### Common Issues

1. **Arduino Not Responding**
   - Check wiring connections
   - Verify Arduino code is uploaded
   - Check baud rate settings (9600)

2. **Blynk Connection Failed**
   - Verify auth token
   - Check APN settings
   - Confirm SIM card data plan
   - Check signal strength

3. **Pump Not Starting**
   - Check Arduino connection status (V10)
   - Verify pump hardware connections
   - Check flow rate setting (must be > 0)

4. **VOC Auto-Trigger Not Working**
   - Verify main ESP32 Bluetooth connection (V11)
   - Check VOC threshold settings
   - Ensure auto-trigger is enabled (V2)
   - Verify main ESP32 has VOC sensor working

### Debug Mode
- Enable serial monitor at 115200 baud
- Check debug output for detailed operation logs
- Monitor JSON command transmission

## Dependencies

### Arduino Libraries Required
- ArduinoJson
- BlynkSimpleTinyGSM
- TinyGsmClient
- TimeLib
- esp_task_wdt

### Hardware Requirements
- ESP32 development board
- SIM7600X cellular modem
- Arduino Mega 2560
- Air pumps (5V compatible)
- Relay modules or transistors for pump control
- VOC sensor (optional, for auto-trigger)

## Future Enhancements

- Data logging to SD card
- Multiple pump support
- Advanced scheduling options
- Sensor data integration
- Remote firmware updates
- Battery monitoring
- GPS location tracking 