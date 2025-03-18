#include "debugConfig.h"
#include "comm.hpp"
#include "SoftwareSerial.h"
#include <Wire.h>

#define SLAVE_ADDRESS 0x08 // Address of this slave device
#define BUFFER_SIZE 32


commandPacket cmd;
telemetryPacket data;


void setup() {
  Wire.begin(SLAVE_ADDRESS); // Join the I2C bus as a slave with the specified address
  Wire.onReceive([](uint8_t numBytes){ receiveEvent(numBytes); }); // Register event for receiving data
  Wire.onRequest(requestEvent); // Register event for sending data
  Serial.begin(9600); // Start serial communication for debugging
  data.robotYawDegrees = 1;
  data.robotDistanceCm = 99;
  data.ultrasonicDistanceCm = 200;
}


void loop() {
  // data.sendUartBytes();
  // data.sendUartASCII();  
  // cmd.readUartBytes();
  delay(100);

  // cmd.readUartASCII();
  // cmd.readI2CASCII(uint8_t address);
  // data.readUartBytes();
  // delay(100);
}


void receiveEvent(uint8_t howMany) {
  cmd.readI2CBytes(howMany);
  // cmd.readI2CASCII(howMany);
}


void requestEvent() {
  data.sendI2CBytes();
  // data.sendI2CASCII();
}