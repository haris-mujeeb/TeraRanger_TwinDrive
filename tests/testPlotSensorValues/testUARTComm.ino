#include <SoftwareSerial.h>
#include "comm.hpp"

#define rxPin 2
#define txPin 3
SoftwareSerial sensorSerial(rxPin, txPin); // RX, TX for TerraRanger

commandPacket cmdData;
telemetryPacket teleData;

const int bufferSize = 64;
char commandBuffer[bufferSize];
int bufferIndex = 0;

void readSerialData();
void parseCommand(char* command);
void forwardTelemetryData();
void readTerraRangerData();

void setup() {
  Wire.begin();
  Serial.begin(9600);
  sensorSerial.begin(115200); // Set baud rate for TerraRanger
}

void loop() {
  readSerialData();
  forwardTelemetryData();
  readTerraRangerData();
}

void readSerialData() {
  while (Serial.available() > 0) {
    char rc = Serial.read();
    if (rc == '\r' || rc == '\n') {
      if (bufferIndex > 0) {
        commandBuffer[bufferIndex] = '\0';
        parseCommand(commandBuffer);
        bufferIndex = 0;
        DEBUG_PRINT(DEBUG_COMM, "Received: " + rc)
      }
    } else if (bufferIndex < bufferSize - 1) {
      commandBuffer[bufferIndex++] = rc;
    } else {
      DEBUG_PRINT(DEBUG_COMM, "Error: Command buffer overflow");
      bufferIndex = 0;
    }
  }
}

void parseCommand(char* command) {
  String commandStr = String(command);
  commandStr.trim();

  int comma1 = commandStr.indexOf(',');
  int comma2 = commandStr.indexOf(',', comma1 + 1);
  
  if (comma1 != -1 && comma2 != -1) {
    String cmd = commandStr.substring(0, comma1);
    String val = commandStr.substring(comma1 + 1, comma2);
    String speed = commandStr.substring(comma2 + 1);
    
    cmd.trim();
    val.trim();
    speed.trim();

    if (cmd == "TURN") {
      cmdData.command = TumblerCommand::Rotate;
      cmdData.commandValue = val.toInt();
      cmdData.commandSpeed = speed.toInt();
      cmdData.sendI2CBytes(SLAVE_ADDR);
      DEBUG_PRINT(DEBUG_COMM, "Turn command recieved: " + String(val) + String(speed));
    }
    else if (cmd == "MOVE") {
      cmdData.command = TumblerCommand::Move;
      cmdData.commandValue = val.toInt();
      cmdData.commandSpeed = speed.toInt(); 
      cmdData.sendI2CBytes(SLAVE_ADDR);
      DEBUG_PRINT(DEBUG_COMM, "Move command recieved: " + String(val) + String(speed));
    } else {
        DEBUG_PRINT(DEBUG_COMM, "ERROR: Invalid MOVE command");
      }
  } else {
    DEBUG_PRINT(DEBUG_COMM, "Unknown command");  
  }
}

void forwardTelemetryData() {  
  teleData.readI2CBytes(SLAVE_ADDR);
  String data = String(teleData.robotYawDegrees) + "," + String(teleData.robotDistanceCm) + "," + String(teleData.ultrasonicDistanceCm) + "\n";
  Serial.print("RT\t");
  Serial.println(data);   // Send data to the Python server when requested
}

void readTerraRangerData(){
  if (sensorSerial.available()) {
    String tof_data = sensorSerial.readStringUntil('\n'); // Read until newline
    Serial.println(tof_data);
  }
}