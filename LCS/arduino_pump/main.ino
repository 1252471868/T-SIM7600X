/*
  =============
  == Summary ==
  =============

  This is the Arduino Mega 2560 pump controller code for the ESP32 pump system.
  It receives JSON commands from ESP32 via Serial2 and controls air pumps and valve.
  Supports duty cycle control for pump flow rate.

  Based on arduino_main communication pattern.
  
  Debug Mode: Send commands via Serial monitor for testing
  - "debug on" / "debug off" - Enable/disable debug mode
  - "inflate <duty>" - Start inflation (e.g., "inflate 200")
  - "deflate <duty>" - Start deflation (e.g., "deflate 150") 
  - "stop" - Stop all pumps
  - "status" - Print current status
*/

// Include libraries
#include <ArduinoJson.h>

// Command definitions for ESP32 communication (matching arduino_main pattern)
#define CMD_INFO "INFO"           // Check communication status
#define CMD_ACK "ACK"             // Acknowledgment
#define CMD_FAIL "FAIL"           // Command failed

// Pump-specific command definitions
#define CMD_INFLATE "INFLATE"     // Inflate command
#define CMD_DEFLATE "DEFLATE"     // Deflate command
#define CMD_STOP "STOP"           // Stop command

// Pin Definitions for Arduino Mega
#define PUMP1_PIN 2               // Digital pin for Pump 1 ON/OFF control
#define PUMP2_PIN 3               // Digital pin for Pump 2 ON/OFF control  
#define VALVE_PIN 4               // Digital pin for Valve control

// PWM Configuration for Pumps
#define PUMP1_PWM_PIN 5           // PWM pin for Pump 1 speed control
#define PUMP2_PWM_PIN 6           // PWM pin for Pump 2 speed control

// Communication control flags and timing
bool ESP32Status = false;                  // Tracks if communication with ESP32 is active
unsigned long lastEspCommandTime = 0;      // Timestamp of the last received command from ESP32
unsigned long lastCommunicationCheck = 0; // Timestamp of the last communication check
const unsigned long COMM_TIMEOUT = 60000;  // ESP32 communication timeout (60 seconds)

// Pump status variables
bool pumpRunning = false;
String currentOperation = "";
int currentDutyCycle = 0;

// Debug mode variables
bool debugMode = false;                    // Debug mode flag
String debugCommand = "";                  // Buffer for debug commands

/**
 * @brief Processes debug commands from Serial monitor
 */
void processDebugCommands()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command.length() == 0) return;
    
    Serial.print("Debug command received: ");
    Serial.println(command);
    
    if (command == "debug on")
    {
      debugMode = true;
      Serial.println("DEBUG MODE ENABLED - Manual pump control active");
      Serial.println("Available commands:");
      Serial.println("  inflate <duty>  - Start inflation (e.g., 'inflate 200')");
      Serial.println("  deflate <duty>  - Start deflation (e.g., 'deflate 150')");
      Serial.println("  stop           - Stop all pumps");
      Serial.println("  status         - Print current status");
      Serial.println("  debug off      - Disable debug mode");
    }
    else if (command == "debug off")
    {
      debugMode = false;
      Serial.println("DEBUG MODE DISABLED - ESP32 control restored");
      stopAllPumpsAndValve(); // Safety: stop pumps when exiting debug mode
    }
    else if (debugMode)
    {
      // Process debug commands only when debug mode is enabled
      if (command.startsWith("inflate"))
      {
        int spaceIndex = command.indexOf(' ');
        int dutyCycle = 255; // Default duty cycle
        
        if (spaceIndex > 0)
        {
          String dutyStr = command.substring(spaceIndex + 1);
          dutyCycle = dutyStr.toInt();
          if (dutyCycle <= 0 || dutyCycle > 255) dutyCycle = 255;
        }
        
        executeInflate(dutyCycle);
        Serial.print("DEBUG: Inflation started with duty cycle ");
        Serial.println(dutyCycle);
      }
      else if (command.startsWith("deflate"))
      {
        int spaceIndex = command.indexOf(' ');
        int dutyCycle = 255; // Default duty cycle
        
        if (spaceIndex > 0)
        {
          String dutyStr = command.substring(spaceIndex + 1);
          dutyCycle = dutyStr.toInt();
          if (dutyCycle <= 0 || dutyCycle > 255) dutyCycle = 255;
        }
        
        executeDeflate(dutyCycle);
        Serial.print("DEBUG: Deflation started with duty cycle ");
        Serial.println(dutyCycle);
      }
      else if (command == "stop")
      {
        stopAllPumpsAndValve();
        Serial.println("DEBUG: All pumps stopped");
      }
      else if (command == "status")
      {
        printDetailedStatus();
      }
      else
      {
        Serial.println("DEBUG: Unknown command. Available commands:");
        Serial.println("  inflate <duty>, deflate <duty>, stop, status, debug off");
      }
    }
    else
    {
      Serial.println("Debug mode is OFF. Send 'debug on' to enable manual control.");
    }
  }
}

/**
 * @brief Processes commands received from the ESP32 via Serial2.
 *        Parses incoming JSON messages and handles defined commands.
 *        Also checks for communication timeouts.
 */
void processEspCommands()
{
  // Skip ESP32 command processing if debug mode is active
  if (debugMode)
  {
    // Still read and discard ESP32 data to prevent buffer overflow
    while (Serial2.available())
    {
      Serial2.read();
    }
    return;
  }
  
  if (Serial2.available())
  {
    // Wait for a complete JSON object
    String jsonString = "";
    bool foundStart = false;
    bool foundEnd = false;

    // Clear any garbage data first
    while (Serial2.available() && Serial2.peek() != '{')
    {
      Serial2.read();
    }

    // Read until we find a complete JSON object
    unsigned long startTime = millis();
    while (Serial2.available() && (millis() - startTime < 1000))
    {
      char c = Serial2.read();

      if (c == '{')
      {
        foundStart = true;
        jsonString = "{";
        continue;
      }

      if (foundStart)
      {
        jsonString += c;

        if (c == '}')
        {
          foundEnd = true;
          break;
        }
      }
    }

    // Only process if we have a complete JSON object
    if (foundStart && foundEnd)
    {
      Serial.println("Received JSON: " + jsonString);

      // Parse JSON
      JsonDocument cmdDoc;
      DeserializationError error = deserializeJson(cmdDoc, jsonString);

      if (error)
      {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
      }

      // Extract command and data
      String command = cmdDoc["cmd"].as<String>();
      String data = cmdDoc["data"].as<String>();

      // Update communication timestamp
      lastEspCommandTime = millis();

      // Handle different commands
      if (command == CMD_INFO)
      {
        // Information/communication check
        sendJsonResponse(CMD_ACK, "Pump Controller Ready");
        ESP32Status = true;
        Serial.println("Info request acknowledged");
      }
      else if (command == CMD_INFLATE)
      {
        // Inflate command with duty cycle
        int dutyCycle = data.toInt();
        if (dutyCycle <= 0) dutyCycle = 255; // Default to full speed
        
        executeInflate(dutyCycle);
        Serial.print("Inflate command executed with duty cycle: ");
        Serial.println(dutyCycle);
      }
      else if (command == CMD_DEFLATE)
      {
        // Deflate command with duty cycle
        int dutyCycle = data.toInt();
        if (dutyCycle <= 0) dutyCycle = 255; // Default to full speed
        
        executeDeflate(dutyCycle);
        Serial.print("Deflate command executed with duty cycle: ");
        Serial.println(dutyCycle);
      }
      else if (command == CMD_STOP)
      {
        // Stop command
        stopAllPumpsAndValve();
        Serial.println("Stop command executed");
      }
      else
      {
        Serial.print("Unknown command: ");
        Serial.println(command);
      }
    }
  }

  // Check for ESP32 communication timeout every 5 seconds (only when not in debug mode)
  if (millis() - lastCommunicationCheck > 5000)
  {
    lastCommunicationCheck = millis();

    // Check for timeout
    if (millis() - lastEspCommandTime > COMM_TIMEOUT)
    {
      if (ESP32Status) { // Only print timeout message once
        Serial.println("ESP32 communication timeout.");
        ESP32Status = false; // Mark ESP32 as disconnected
        // Safety: Stop all pumps on communication timeout
        stopAllPumpsAndValve();
      }
    }
  }
}

/**
 * @brief Sends a JSON formatted response to the ESP32 via Serial2.
 * @param cmd The command string (e.g., CMD_ACK, CMD_FAIL).
 * @param data The data payload string (can be empty).
 */
void sendJsonResponse(const String &cmd, const String &data)
{
  JsonDocument respDoc;
  respDoc["cmd"] = cmd;
  respDoc["data"] = data;

  String response;
  serializeJson(respDoc, response);
  Serial2.println(response);

  Serial.print("Sent response: ");
  Serial.println(response);
}

/**
 * @brief Execute inflate operation with specified duty cycle
 * @param dutyCycle PWM duty cycle (0-255) for pump speed control
 */
void executeInflate(int dutyCycle)
{
  // Turn on both pumps with specified duty cycle for inflation
  digitalWrite(PUMP1_PIN, HIGH);
  digitalWrite(PUMP2_PIN, HIGH);
  analogWrite(PUMP1_PWM_PIN, dutyCycle);
  analogWrite(PUMP2_PWM_PIN, dutyCycle);
  
  // Turn on valve for inflation path
  digitalWrite(VALVE_PIN, HIGH);
  
  // Update status
  pumpRunning = true;
  currentOperation = "INFLATE";
  currentDutyCycle = dutyCycle;
  
  Serial.print("Pumps inflating at duty cycle: ");
  Serial.println(dutyCycle);
}

/**
 * @brief Execute deflate operation with specified duty cycle
 * @param dutyCycle PWM duty cycle (0-255) for pump speed control
 */
void executeDeflate(int dutyCycle)
{
  // Turn on both pumps with specified duty cycle for deflation
  digitalWrite(PUMP1_PIN, HIGH);
  digitalWrite(PUMP2_PIN, HIGH);
  analogWrite(PUMP1_PWM_PIN, dutyCycle);
  analogWrite(PUMP2_PWM_PIN, dutyCycle);
  
  // Turn off valve for deflation path
  digitalWrite(VALVE_PIN, LOW);
  
  // Update status
  pumpRunning = true;
  currentOperation = "DEFLATE";
  currentDutyCycle = dutyCycle;
  
  Serial.print("Pumps deflating at duty cycle: ");
  Serial.println(dutyCycle);
}

/**
 * @brief Stop all pumps and valve
 */
void stopAllPumpsAndValve()
{
  // Turn off all pumps and valve
  digitalWrite(PUMP1_PIN, LOW);
  digitalWrite(PUMP2_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
  analogWrite(PUMP1_PWM_PIN, 0);
  analogWrite(PUMP2_PWM_PIN, 0);
  
  // Update status
  pumpRunning = false;
  currentOperation = "STOPPED";
  currentDutyCycle = 0;
  
  Serial.println("All pumps and valve stopped");
}

/**
 * @brief Print current pump status to Serial monitor
 */
void printStatus()
{
  Serial.print("ESP32 Status: ");
  Serial.print(ESP32Status ? "Connected" : "Disconnected");
  Serial.print(" | Pump Status: ");
  Serial.print(pumpRunning ? "Running" : "Stopped");
  Serial.print(" | Operation: ");
  Serial.print(currentOperation);
  Serial.print(" | Duty Cycle: ");
  Serial.print(currentDutyCycle);
  Serial.print(" | Debug Mode: ");
  Serial.println(debugMode ? "ON" : "OFF");
}

/**
 * @brief Print detailed status information for debug mode
 */
void printDetailedStatus()
{
  Serial.println("=== PUMP CONTROLLER STATUS ===");
  Serial.print("Debug Mode: ");
  Serial.println(debugMode ? "ENABLED" : "DISABLED");
  Serial.print("ESP32 Communication: ");
  Serial.println(ESP32Status ? "Connected" : "Disconnected");
  Serial.print("Pump Status: ");
  Serial.println(pumpRunning ? "Running" : "Stopped");
  Serial.print("Current Operation: ");
  Serial.println(currentOperation);
  Serial.print("Duty Cycle: ");
  Serial.print(currentDutyCycle);
  Serial.println(" (0-255)");
  
  Serial.println("\nPin States:");
  Serial.print("  PUMP1_PIN (2): ");
  Serial.println(digitalRead(PUMP1_PIN) ? "HIGH" : "LOW");
  Serial.print("  PUMP2_PIN (3): ");
  Serial.println(digitalRead(PUMP2_PIN) ? "HIGH" : "LOW");
  Serial.print("  VALVE_PIN (4): ");
  Serial.println(digitalRead(VALVE_PIN) ? "HIGH" : "LOW");
  
  Serial.print("Last ESP32 Command: ");
  Serial.print((millis() - lastEspCommandTime) / 1000);
  Serial.println(" seconds ago");
  Serial.println("==============================");
}

/**
 * @brief Initializes hardware, communication, and pins.
 *        Runs once at startup.
 */
void setup()
{
  // Begin serial communication
  Serial.begin(115200);   // Debug serial
  Serial2.begin(9600);    // ESP32 communication (matching config)
  delay(500);
  Serial.println("Arduino Mega Pump Controller Initializing...");

  // Update communication timestamp
  lastEspCommandTime = millis();

  // Initialize pump and valve pins
  pinMode(PUMP1_PIN, OUTPUT);
  pinMode(PUMP2_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  
  // Initialize PWM pins for speed control
  pinMode(PUMP1_PWM_PIN, OUTPUT);
  pinMode(PUMP2_PWM_PIN, OUTPUT);
  
  // Ensure everything is off initially
  stopAllPumpsAndValve();
  
  Serial.println("Pump controller pins initialized");
  Serial.println("Pin Configuration:");
  Serial.println("  PUMP1_PIN (ON/OFF): 2");
  Serial.println("  PUMP2_PIN (ON/OFF): 3");
  Serial.println("  VALVE_PIN: 4");
  Serial.println("  PUMP1_PWM_PIN (Speed): 5");
  Serial.println("  PUMP2_PWM_PIN (Speed): 6");
  Serial.println("Ready to receive commands from ESP32");
  
  Serial.println("\n=== DEBUG MODE AVAILABLE ===");
  Serial.println("Send 'debug on' to enable manual pump control");
  Serial.println("Send 'debug off' to disable manual control");
  Serial.println("============================\n");
  
  // Print initial status
  printStatus();
}

/**
 * @brief Main loop. Continuously processes ESP32 commands and monitors status.
 */
void loop()
{
  // Process debug commands from Serial monitor
  processDebugCommands();
  
  // Process any incoming commands from ESP32 (unless in debug mode)
  processEspCommands();
  
  // Print status every 10 seconds for debugging
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 10000)
  {
    lastStatusPrint = millis();
    printStatus();
  }
  
  // Small delay for stability
  delay(10);
}
