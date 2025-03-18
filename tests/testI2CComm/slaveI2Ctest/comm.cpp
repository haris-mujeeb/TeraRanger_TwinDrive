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
  snprintf(buffer, BUFFER_SIZE, "%hd,%ld,%hd", robotYawDegrees, robotDistanceCm, ultrasonicDistanceCm);
  Wire.write(buffer);
  DEBUG_PRINT(DEBUG_COMM, "Sent I2C: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
}

void telemetryPacket::sendUartBytes() const {
  Serial.write(reinterpret_cast<const uint8_t*>(&robotYawDegrees), sizeof(robotYawDegrees));
  Serial.write(reinterpret_cast<const uint8_t*>(&robotDistanceCm), sizeof(robotDistanceCm));
  Serial.write(reinterpret_cast<const uint8_t*>(&ultrasonicDistanceCm), sizeof(ultrasonicDistanceCm));
  DEBUG_PRINT(DEBUG_COMM, "Sent Uart: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
}

void telemetryPacket::sendUartASCII() const {
  char buffer[BUFFER_SIZE];
  snprintf(buffer, BUFFER_SIZE, "%hd,%ld,%hd\n", robotYawDegrees, robotDistanceCm, ultrasonicDistanceCm);
  Serial.println(buffer);
  DEBUG_PRINT(DEBUG_COMM, "Sent Uart: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
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

void telemetryPacket::readUartBytes() {
  Serial.readBytes(reinterpret_cast<uint8_t*>(&robotYawDegrees), sizeof(robotYawDegrees));
  Serial.readBytes(reinterpret_cast<uint8_t*>(&robotDistanceCm), sizeof(robotDistanceCm));
  Serial.readBytes(reinterpret_cast<uint8_t*>(&ultrasonicDistanceCm), sizeof(ultrasonicDistanceCm));
  DEBUG_PRINT(DEBUG_COMM, "Recv Uart: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
}

void telemetryPacket::readUartASCII() {
  String input = Serial.readStringUntil('\n');
  sscanf(input.c_str(), "%hd,%ld,%hd", &robotYawDegrees, &robotDistanceCm, &ultrasonicDistanceCm);
  DEBUG_PRINT(DEBUG_COMM, "Recv Uart: " + String(robotYawDegrees) + "," + String(robotDistanceCm) + "," + String(ultrasonicDistanceCm));
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

void commandPacket::sendUartBytes() const {
  Serial.write(command);
  Serial.write(reinterpret_cast<const uint8_t*>(&commandValue), sizeof(commandValue));
  DEBUG_PRINT(DEBUG_COMM, "Sent Uart: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::sendUartASCII() const {
  char buffer[BUFFER_SIZE];
  snprintf(buffer, BUFFER_SIZE, "%hhu,%d,%hhu\n", command, commandValue, commandSpeed);
  Serial.println(buffer);
  DEBUG_PRINT(DEBUG_COMM, "Sent Uart: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::readI2CBytes(uint8_t numBytes) {
  if (numBytes < 1) {
    Serial.println("Error: No data received.");
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

void commandPacket::readUartBytes() {
  command = static_cast<TumblerCommand>(Serial.read());
  Serial.readBytes(reinterpret_cast<uint8_t*>(&commandValue), sizeof(commandValue));
  Serial.readBytes(reinterpret_cast<uint8_t*>(&commandSpeed), sizeof(commandSpeed));
  DEBUG_PRINT(DEBUG_COMM, "Recv Uart: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}

void commandPacket::readUartASCII() {
  String input = Serial.readStringUntil('\n');
  uint8_t tempCommand;
  sscanf(input.c_str(), "%hhu,%hd,%hhu", &tempCommand, &commandValue, &commandSpeed);
    if (tempCommand >= INVALID) {
      command = INVALID;
  } else {
      command = static_cast<TumblerCommand>(tempCommand);
  }
  DEBUG_PRINT(DEBUG_COMM, "Recv Uart: " + commandToString(command) + "," + String(commandValue) + "," + String(commandSpeed));
}