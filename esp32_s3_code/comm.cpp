#include "comm.hpp"

TumblerCommand stringToCommand(const String& commandStr) {
  if (commandStr == "Stop") return Stop;
  if (commandStr == "Move") return Move;
  if (commandStr == "Rotate") return Rotate;
  return INVALID;
}

String commandToString(TumblerCommand cmd) {
  switch (cmd) {
    case Stop: return "Stop";
    case Move: return "Move";
    case Rotate: return "Rotate";
    default: return "INVALID";   
  }
}

void telemetryPacket::sendI2CBytes() const {
  Wire.write(reinterpret_cast<const uint8_t*>(&robotYawDegrees), sizeof(robotYawDegrees));
  Wire.write(reinterpret_cast<const uint8_t*>(&robotDistanceCm), sizeof(robotDistanceCm));
  Wire.write(reinterpret_cast<const uint8_t*>(&ultrasonicDistanceCm), sizeof(ultrasonicDistanceCm));
  DEBUG_PRINT(DEBUG_COMM, "Sent I2C: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
}

void telemetryPacket::sendI2CASCII() const {
  char buffer[BUFFER_SIZE];
  int snprintfResult = snprintf(buffer, BUFFER_SIZE, "%hd,%ld,%hd", 
                                robotYawDegrees, robotDistanceCm, ultrasonicDistanceCm);

  if (snprintfResult >= 0 && snprintfResult < BUFFER_SIZE) { // Check for successful formatting
    Wire.beginTransmission(SLAVE_ADDR);
    for (int i = 0; buffer[i] != '\0'; i++) {
      Wire.write(static_cast<uint8_t>(buffer[i]));
    }
    Wire.endTransmission();
    DEBUG_PRINT(DEBUG_COMM, "Sent I2C: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
  } else {
    DEBUG_PRINT(DEBUG_COMM, "Error formatting I2C data.");
  }
}

void telemetryPacket::sendUartBytes(Stream& serial) const {
  serial.write(reinterpret_cast<const uint8_t*>(&robotYawDegrees), sizeof(robotYawDegrees));
  serial.write(reinterpret_cast<const uint8_t*>(&robotDistanceCm), sizeof(robotDistanceCm));
  serial.write(reinterpret_cast<const uint8_t*>(&ultrasonicDistanceCm), sizeof(ultrasonicDistanceCm));
  DEBUG_PRINT(DEBUG_COMM, "Sent Uart: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
}

void telemetryPacket::sendUartASCII(Stream& serial) const {
  char buffer[BUFFER_SIZE];
  snprintf(buffer, BUFFER_SIZE, "%hd,%ld,%hhu,%d,%d,%ld,%ld",
           robotYawDegrees,
           robotDistanceCm,
           ultrasonicDistanceCm,
           leftIR_Detected,
           rightIR_Detected,
           leftMotorEncoderValue,
           rightMotorEncoderValue);

  serial.println(buffer);

  DEBUG_PRINT(DEBUG_COMM, "Sent Uart: " + String(robotYawDegrees) + "," +
                                       String(robotDistanceCm) + "," +
                                       String(ultrasonicDistanceCm) + "," +
                                       String(leftIR_Detected) + "," +
                                       String(rightIR_Detected) + "," +
                                       String(leftMotorEncoderValue) + "," +
                                       String(rightMotorEncoderValue));
}

void telemetryPacket::readI2CBytes(uint8_t address) {
  Wire.requestFrom(address, (uint8_t)(sizeof(robotYawDegrees) + sizeof(robotDistanceCm) + sizeof(ultrasonicDistanceCm)));
  if (Wire.available() < sizeof(robotYawDegrees) + sizeof(robotDistanceCm) + sizeof(ultrasonicDistanceCm)) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient bytes received.");
    return;  // Insufficient bytes received
  }
  Wire.readBytes(reinterpret_cast<uint8_t*>(&robotYawDegrees), sizeof(robotYawDegrees));
  Wire.readBytes(reinterpret_cast<uint8_t*>(&robotDistanceCm), sizeof(robotDistanceCm));
  Wire.readBytes(reinterpret_cast<uint8_t*>(&ultrasonicDistanceCm), sizeof(ultrasonicDistanceCm));
  DEBUG_PRINT(DEBUG_COMM, "Recv I2C: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
}

void telemetryPacket::readI2CASCII(uint8_t address) {
  char buffer[BUFFER_SIZE];
  Wire.requestFrom(address, (uint8_t)(BUFFER_SIZE - 1));
  int index = 0;
  while (Wire.available() && index < BUFFER_SIZE - 1) {
    buffer[index++] = Wire.read();
  }
  buffer[index] = '\0';
  sscanf(buffer, "%hd,%ld,%hd", &robotYawDegrees, &robotDistanceCm, &ultrasonicDistanceCm);
  DEBUG_PRINT(DEBUG_COMM, "Recv I2C: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
}

void telemetryPacket::readUartBytes(Stream& serial) {
  serial.readBytes(reinterpret_cast<uint8_t*>(&robotYawDegrees), sizeof(robotYawDegrees));
  serial.readBytes(reinterpret_cast<uint8_t*>(&robotDistanceCm), sizeof(robotDistanceCm));
  serial.readBytes(reinterpret_cast<uint8_t*>(&ultrasonicDistanceCm), sizeof(ultrasonicDistanceCm));
  DEBUG_PRINT(DEBUG_COMM, "Recv Uart: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
}

void telemetryPacket::readUartASCII(Stream& serial) {
  String input = serial.readStringUntil('\n');
  sscanf(input.c_str(), "%hd,%ld,%hhu,%d,%d,%hd,%hd",
         &robotYawDegrees,
         &robotDistanceCm,
         &ultrasonicDistanceCm,
         (int*)&leftIR_Detected,
         (int*)&rightIR_Detected,
         &leftMotorEncoderValue,
         &rightMotorEncoderValue);

  DEBUG_PRINT(DEBUG_COMM, "Recv Raw: " + input);
  DEBUG_PRINT(DEBUG_COMM, "Recv Uart: " +
              String(robotYawDegrees) + "," +
              String(robotDistanceCm) + "," +
              String(ultrasonicDistanceCm) + "," +
              String(leftIR_Detected) + "," +
              String(rightIR_Detected) + "," +
              String(leftMotorEncoderValue) + "," +
              String(rightMotorEncoderValue));
}

void commandPacket::sendI2CBytes(uint8_t address) const {
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(reinterpret_cast<const uint8_t*>(&commandValue), sizeof(commandValue));
  Wire.write(reinterpret_cast<const uint8_t*>(&commandSpeed), sizeof(commandSpeed));
  Wire.endTransmission();
  DEBUG_PRINT(DEBUG_COMM, "Sent I2C: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::sendI2CASCII(uint8_t address) const {
  Wire.beginTransmission(address);
  char buffer[BUFFER_SIZE];
  snprintf(buffer, BUFFER_SIZE, "%hhu,%d,%hhu", command, commandValue, commandSpeed);
  Wire.print(buffer);
  Wire.endTransmission();
  DEBUG_PRINT(DEBUG_COMM, "Sent I2C: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::sendUartBytes(Stream& serial) const {
  serial.write(command);
  serial.write(reinterpret_cast<const uint8_t*>(&commandValue), sizeof(commandValue));
  DEBUG_PRINT(DEBUG_COMM, "Sent Uart: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::sendUartASCII(Stream& serial) const {
  char buffer[BUFFER_SIZE];
  snprintf(buffer, BUFFER_SIZE, "%hhu,%d,%hhu\n", command, commandValue, commandSpeed);
  serial.println(buffer);
  DEBUG_PRINT(DEBUG_COMM, "Sent Uart: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::readI2CBytes(uint8_t numBytes) {
  if (numBytes < 1) {
    ERROR_PRINT("Error: No data received.");
    return;
  }

  command = static_cast<TumblerCommand>(Wire.read());

  if (command < 0 || command >= INVALID) {
    command = INVALID;  // Default to Stop
    DEBUG_PRINT(DEBUG_COMM, "Error: Invalid command received.");
    return;
  }
  Wire.readBytes(reinterpret_cast<uint8_t*>(&commandValue), sizeof(commandValue));
  Wire.readBytes(reinterpret_cast<uint8_t*>(&commandSpeed), sizeof(commandSpeed));
  DEBUG_PRINT(DEBUG_COMM, "Recv I2C: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::readI2CASCII(uint8_t address) {
  char buffer[BUFFER_SIZE];
  Wire.requestFrom(address, (uint8_t)(BUFFER_SIZE - 1));
  int index = 0;
  while (Wire.available() && index < BUFFER_SIZE - 1) {
    buffer[index++] = Wire.read();
  }
  buffer[index] = '\0';
  sscanf(buffer, "%hhu,%hd,%hhu", &command, &commandValue, &commandSpeed);
  DEBUG_PRINT(DEBUG_COMM, "Recv I2C: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::readUartBytes(Stream& serial) {
  command = static_cast<TumblerCommand>(serial.read());
  serial.readBytes(reinterpret_cast<uint8_t*>(&commandValue), sizeof(commandValue));
  serial.readBytes(reinterpret_cast<uint8_t*>(&commandSpeed), sizeof(commandSpeed));
  DEBUG_PRINT(DEBUG_COMM, "Recv Uart: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::readUartASCII(Stream& serial) {
  String input = serial.readStringUntil('\n');
  uint8_t tempCommand;
  sscanf(input.c_str(), "%hhu,%hd,%hhu", &tempCommand, &commandValue, &commandSpeed);
    if (tempCommand >= INVALID) {
      command = INVALID;
  } else {
      command = static_cast<TumblerCommand>(tempCommand);
  }
  DEBUG_PRINT(DEBUG_COMM, "Recv Uart: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}