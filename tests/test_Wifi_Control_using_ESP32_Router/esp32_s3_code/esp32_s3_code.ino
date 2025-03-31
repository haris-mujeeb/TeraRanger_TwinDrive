// Create a SoftwareSerial object for sensor communication
#include "comm.hpp"


#include <HardwareSerial.h>
HardwareSerial sensorSerial(1);  // Use UART1
HardwareSerial robotSerial(2);  // Use UART2

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
  Serial.begin(9600);
  sensorSerial.begin(115200, SERIAL_8N1, 18, 17);  // RX, TX
  robotSerial.begin(9600, SERIAL_8N1, 16, 15);  // RX, TX
}

unsigned long last_time = 0;
void loop() {
  readSerialData();
  forwardTelemetryData();
  if (millis() - last_time > 100) {  // 115200 baudrate is too high for SoftwareSerial port.
    readTerraRangerData();
    last_time = millis();
  }
}

void readSerialData() {
  `hile (Serial.available() > 0) {
    Serial.print(1);
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
      // cmdData.sendI2CBytes(SLAVE_ADDR);
      cmdData.sendUartASCII(robotSerial);
      DEBUG_PRINT(DEBUG_COMM, "Turn command recieved: " + String(val) + String(speed));
    } else if (cmd == "MOVE") {
      cmdData.command = TumblerCommand::Move;
      cmdData.commandValue = val.toInt();
      cmdData.commandSpeed = speed.toInt();
      // cmdData.sendI2CBytes(SLAVE_ADDR);
      cmdData.sendUartASCII(robotSerial);
      DEBUG_PRINT(DEBUG_COMM, "Move command recieved: " + String(val) + String(speed));
    } else {
      DEBUG_PRINT(DEBUG_COMM, "ERROR: Invalid MOVE command");
    }
  } else {
    DEBUG_PRINT(DEBUG_COMM, "Unknown command");
  }
}

void forwardTelemetryData() {
  // teleData.readI2CBytes(SLAVE_ADDR);
  teleData.readUartASCII(robotSerial);
  String data = String(teleData.robotYawDegrees) + "," + String(teleData.robotDistanceCm) + "," + String(teleData.ultrasonicDistanceCm);
  Serial.print("RT\t");
  Serial.println(data);  // Send data to the Python server when requested
}

void readTerraRangerData() {
  if (sensorSerial.available()) {
    String tof_data = sensorSerial.readStringUntil('\n');  // Read until newline
    Serial.println(tof_data);
  }
}