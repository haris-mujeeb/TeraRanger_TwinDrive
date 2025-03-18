#include "debugConfig.h"
#include "comm.hpp"
#include "SoftwareSerial.h"
#include <Wire.h>

#define SLAVE_ADDRESS 0x08 // Address of this slave device
#define BUFFER_SIZE 32


void setup() {
  Wire.begin(); // Join the I2C bus as a master
  Serial.begin(9600); // Start serial communication for debugging
}

telemetryPacket telemetry_data;
commandPacket command_data;

void loop() {
  command_data.command = Move;
  command_data.commandValue = 100;
  // command_data.sendUartASCII();
  // command_data.sendUartBytes();

  // command_data.sendI2CASCII(SLAVE_ADDRESS);
  command_data.sendI2CBytes(SLAVE_ADDRESS);
  
  // telemetry_data.yaw_ = 100;
  // telemetry_data.distance_ = 123456;
  // telemetry_data.sendUartBytes();

  delay(10); // Wait for a second

  // telemetry_data.readUartASCII();
  // telemetry_data.readUartBytes();

  // telemetry_data.readI2CASCII(SLAVE_ADDRESS);
  telemetry_data.readI2CBytes(SLAVE_ADDRESS);

  delay(10); // Wait for a second
}
