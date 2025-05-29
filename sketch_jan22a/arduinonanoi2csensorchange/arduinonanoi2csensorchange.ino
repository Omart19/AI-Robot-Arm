#include <SoftwareSerial.h>

// Define the pins for SoftwareSerial communication with the TOF sensor
SoftwareSerial tofSerial(2, 3); // Arduino RX pin = 2, Arduino TX pin = 3

// --- Define Commands (Slave ID 01) ---
// Mode Control (Register 0x0009)
byte switchToI2CCommand[] = {0x01, 0x06, 0x00, 0x09, 0x00, 0x01, 0x98, 0x08};
byte switchToUARTCommand[] = {0x01, 0x06, 0x00, 0x09, 0x00, 0x00, 0xD9, 0xCA};

// Special Register 0x0001 Commands
byte testCommunicationCommand[] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD8, 0xDA}; // Value 0x0000 to Reg 0x0001
byte restartModuleCommand[] = {0x01, 0x06, 0x00, 0x01, 0x10, 0x00, 0x19, 0xF9};  // Value 0x1000 to Reg 0x0001 (Reboot)
byte restoreDefaultsCommand[] = {0x01, 0x06, 0x00, 0x01, 0xAA, 0x55, 0xED, 0xDE}; // Value 0xAA55 to Reg 0x0001

// Ranging Mode Control (Register 0x0004)
byte setHighPrecisionMode[] = {0x01, 0x06, 0x00, 0x04, 0x00, 0x01, 0x09, 0xCB};
byte setMiddleDistanceMode[] = {0x01, 0x06, 0x00, 0x04, 0x00, 0x02, 0x48, 0x0B};
byte setLongDistanceMode[] = {0x01, 0x06, 0x00, 0x04, 0x00, 0x03, 0xCA, 0xC9}; // Corrected LSB, MSB order for CRC 0xC9CA

// Data Read (Register 0x0010)
byte readDistanceUARTCommand[] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};

// Auto Output Control (Register 0x0005)
byte setAutoOutput500msCommand[] = {0x01, 0x06, 0x00, 0x05, 0x01, 0xF4, 0x99, 0xDC};
byte stopAutoOutputCommand[] = {0x01, 0x06, 0x00, 0x05, 0x00, 0x00, 0x18, 0x0A};

// --- !!! SELECT SENSOR'S CURRENT UART BAUD RATE HERE !!! ---
// Try 115200 first. If initial async data is garbage, try 9600, then 38400.
long sensorBaudRate = 115200;
//long sensorBaudRate = 9600;
//long sensorBaudRate = 38400;

void setup() {
  Serial.begin(115200); // For PC communication
  while (!Serial) {
    ;
  }
  Serial.println(F("Interactive TOF Sensor Controller (v4.4 - Baud Test)"));
  Serial.println(F("Ensure sensor is wired for UART (RX/TX to D2/D3), 5V, GND."));
  Serial.println(F("Set Serial Monitor Line Ending to 'No line ending'."));
  Serial.print(F("Attempting to communicate with sensor at "));
  Serial.print(sensorBaudRate);
  Serial.println(F(" bps..."));
  
  tofSerial.begin(sensorBaudRate);
  delay(200); 
  Serial.println(F("Sketch ready. Observe initial async data. Then send commands."));
  Serial.println(F("----------------MENU------------------------------------"));
  Serial.println(F("  '1' = Switch to I2C mode"));
  Serial.println(F("  '2' = Switch to UART mode (ensures reg 0x0009=0)"));
  Serial.println(F("  --- Special Register 0x0001 Commands ---"));
  Serial.println(F("  'T' = Test Communication (writes 0x0000 to reg 0x0001)"));
  Serial.println(F("  'R' = Reboot Sensor (writes 0x1000 to reg 0x0001)"));
  Serial.println(F("  'D' = Restore Default Parameters (writes 0xAA55 to reg 0x0001)"));
  Serial.println(F("  --- General UART Commands ---"));
  Serial.println(F("  '4' = Read Distance (from reg 0x0010, uses current range mode)"));
  Serial.println(F("  --- Set UART Ranging Mode (then use '4' to read) ---"));
  Serial.println(F("  '6' = Set High Precision Mode (0.2m)"));
  Serial.println(F("  '7' = Set Middle Distance Mode (0.4m)"));
  Serial.println(F("  '8' = Set Long Distance Mode (0.5m)"));
  Serial.println(F("  --- UART Auto Output ---"));
  Serial.println(F("  'A' = Set Auto Output (500ms, uses current range mode)"));
  Serial.println(F("  'S' = Stop Auto Output"));
  Serial.println(F("----------------------------------------------------"));
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    while (Serial.available() > 0) { Serial.read(); } 

    if (command == '1') {
      Serial.println(F("Sending: Switch to I2C mode..."));
      sendCommandToSensor(switchToI2CCommand, sizeof(switchToI2CCommand));
    } else if (command == '2') {
      Serial.println(F("Sending: Switch to UART mode (reg 0x0009 = 0)..."));
      sendCommandToSensor(switchToUARTCommand, sizeof(switchToUARTCommand));
    } else if (command == 'T' || command == 't') { 
      Serial.println(F("Sending: Test Communication (reg 0x0001 = 0x0000)..."));
      sendCommandToSensor(testCommunicationCommand, sizeof(testCommunicationCommand));
      Serial.println(F("Expected echo: 01 06 00 01 00 00 D8 DA"));
    } else if (command == 'R' || command == 'r') { 
      Serial.println(F("Sending: REBOOT module (reg 0x0001 = 0x1000)..."));
      sendCommandToSensor(restartModuleCommand, sizeof(restartModuleCommand)); // Changed to restartModuleCommand
    } else if (command == 'D' || command == 'd') { 
      Serial.println(F("Sending: Restore Default Parameters (reg 0x0001 = 0xAA55)..."));
      sendCommandToSensor(restoreDefaultsCommand, sizeof(restoreDefaultsCommand));
    } else if (command == '4') {
      Serial.println(F("Sending: Read Distance via UART/Modbus..."));
      sendCommandToSensor(readDistanceUARTCommand, sizeof(readDistanceUARTCommand));
      Serial.println(F("Expected response: 01 03 02 XX XX CR CH (XX XX is distance)"));
    } else if (command == '6') {
      Serial.println(F("Sending: Set High Precision Mode (0.2m)..."));
      sendCommandToSensor(setHighPrecisionMode, sizeof(setHighPrecisionMode));
      Serial.println(F("Expected echo: 01 06 00 04 00 01 09 CB"));
    } else if (command == '7') {
      Serial.println(F("Sending: Set Middle Distance Mode (0.4m)..."));
      sendCommandToSensor(setMiddleDistanceMode, sizeof(setMiddleDistanceMode));
      Serial.println(F("Expected echo: 01 06 00 04 00 02 48 0B"));
    } else if (command == '8') {
      Serial.println(F("Sending: Set Long Distance Mode (0.5m)..."));
      sendCommandToSensor(setLongDistanceMode, sizeof(setLongDistanceMode));
      Serial.println(F("Expected echo: 01 06 00 04 00 03 CA C9"));
    } else if (command == 'A' || command == 'a') {
      Serial.println(F("Sending: Set Auto Output 500ms..."));
      sendCommandToSensor(setAutoOutput500msCommand, sizeof(setAutoOutput500msCommand));
      Serial.println(F("Expected echo: 01 06 00 05 01 F4 99 DC"));
    } else if (command == 'S' || command == 's') {
      Serial.println(F("Sending: Stop Auto Output..."));
      sendCommandToSensor(stopAutoOutputCommand, sizeof(stopAutoOutputCommand));
      Serial.println(F("Expected echo: 01 06 00 05 00 00 18 0A"));
    }
    else {
      Serial.print(F("Unknown command: '")); 
      Serial.print(command);
      Serial.println(F("'. See menu for options."));
    }
    Serial.println(F("----------------------------------------------------"));
  }

  // Listen for any data from the sensor
  if (tofSerial.available() > 0) {
    Serial.print(F("Async data from sensor: "));
    while (tofSerial.available() > 0) {
      byte b = tofSerial.read();
      if (b < 0x10) Serial.print("0"); 
      Serial.print(b, HEX);
      Serial.print(" ");
    }
    Serial.println(); 
  }
}

void sendCommandToSensor(byte cmd[], int cmdLength) {
  // Clear any residual data from tofSerial RX buffer before sending
  while(tofSerial.available()) {
    tofSerial.read();
  }

  tofSerial.write(cmd, cmdLength); 
  Serial.println(F("Command sent to sensor."));

  delay(300); 
  String response = "";
  bool gotResponse = false;
  while (tofSerial.available() > 0) {
    gotResponse = true;
    byte b = tofSerial.read();
    if (b < 0x10) response += "0"; 
    response += String(b, HEX);
    response += " ";
    // Removed delayMicroseconds here to capture response faster if it's quick
  }

  if (gotResponse) {
    Serial.print(F("Immediate response from sensor: "));
    response.trim(); 
    response.toUpperCase(); 
    Serial.println(response); 
  } else {
    Serial.println(F("No immediate response received from sensor. (This can be normal)."));
  }
}