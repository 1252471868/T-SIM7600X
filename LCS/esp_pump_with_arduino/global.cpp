#include "global.h"

// TinyGSM and hardware objects
TinyGsm modem(SerialAT);
HardwareSerial ArduinoSerial(2);  // Using Serial2 for Arduino communication

// Network and connection status
volatile bool internetAvailable = false;
bool blynkConnected = false;

// Pump system global variables
PumpOperation pumpOp = {
  .enabled = false,
  .flowRate = 255,              // Default to full speed
  .mode = PUMP_OFF,
  .startTime = 0,
  .duration = 0,
  .isInflating = true           // Default to inflation mode
};

VOCMonitoring vocMonitor = {
  .autoEnabled = false,
  .threshold = 100.0,           // Default VOC threshold
  .currentLevel = 0.0,
  .lastCheck = 0
};

TimedOperation timedOp = {
  .enabled = false,
  .startTime = 0,
  .samplingDuration = 30000,    // Default 30 seconds
  .scheduled = false
};

// Communication status
unsigned long lastArduinoComm = 0;
bool arduinoConnected = false;
unsigned long lastMainESP32Comm = 0;
bool mainESP32Connected = false;

// Timer variables
unsigned long lastStatusUpdate = 0;
unsigned long lastVOCCheck = 0; 