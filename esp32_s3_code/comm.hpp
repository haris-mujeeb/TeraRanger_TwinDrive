// telemetry_command.hpp

#ifndef TELEMETRY_COMMAND_HPP
#define TELEMETRY_COMMAND_HPP

#include <stdint.h>
#include "Arduino.h"
#include <Wire.h>
#include "debugConfig.h"

#define SLAVE_ADDR 8
#define BUFFER_SIZE 64

/**
 * @brief Checks if the given string represents a numeric value.
 * 
 * @param str The input string to be checked.
 * @return true if the string is numeric, false otherwise.
 */
bool isNumeric(const String& str);

/**
 * @brief Enumeration for various tumbler commands.
 */
enum TumblerCommand : uint8_t {
  Stop,        ///< Command to stop the tumbler.
  Move,        ///< Command to move the tumbler forward.
  Rotate,      ///< Command to rotate the tumbler.
  INVALID      ///< Represents an invalid command.
};

/**
 * @brief Converts a command string to its corresponding TumblerCommand enum.
 * 
 * @param commandStr The string representation of the command.
 * @return The corresponding TumblerCommand value.
 */
TumblerCommand stringToCommand(const String& commandStr);

/**
 * @brief Converts a TumblerCommand enum to its string representation.
 * 
 * @param cmd The TumblerCommand value.
 * @return The string representation of the command.
 */
String commandToString(TumblerCommand cmd);

/**
 * @brief Structure to handle telemetry data packets.
 */
struct telemetryPacket {
  int16_t robotYawDegrees = 0;      ///< Robot Yaw angle in degrees.
  long robotDistanceCm = 0.0;       ///< Robot distance measurement in centimeters.
  uint8_t ultrasonicDistanceCm = 0; ///< Ultrasonic distance measurement in centimeters.
  bool leftIR_Detected = 0;
  bool rightIR_Detected = 0;
  int32_t leftMotorEncoderValue = 0;      ///< Robot Yaw angle in degrees.
  int32_t rightMotorEncoderValue = 0;      ///< Robot Yaw angle in degrees.

  /**
   * @brief Sends telemetry data as raw bytes over I2C.
   */
  void sendI2CBytes() const;

  /**
   * @brief Sends telemetry data as ASCII over I2C.
   */
  void sendI2CASCII() const;

  /**
   * @brief Sends telemetry data as raw bytes over UART.
   */
  void sendUartBytes(Stream& serial) const;

  /**
   * @brief Sends telemetry data as ASCII over UART.
   */
  void sendUartASCII(Stream& serial) const;

  /**
   * @brief Reads telemetry data as raw bytes from an I2C device.
   * 
   * @param address The I2C address to read from.
   */
  void readI2CBytes(uint8_t address);

  /**
   * @brief Reads telemetry data as ASCII from an I2C device.
   * 
   * @param address The I2C address to read from.
   */
  void readI2CASCII(uint8_t address);

  /**
   * @brief Reads telemetry data as raw bytes from UART.
   */
  void readUartBytes(Stream& serial);

  /**
   * @brief Reads telemetry data as ASCII from UART.
   */
  void readUartASCII(Stream& serial);
};

/**
 * @brief Structure to handle command data packets.
 */
struct commandPacket {
  TumblerCommand command;    ///< Command to be sent.
  int16_t commandValue = 0.0; ///< Value associated with the command.
  uint8_t commandSpeed = 0.0; ///< Speed associated with the command.
  /**
   * @brief Sends command data as raw bytes over I2C.
   * 
   * @param address The I2C address to send to.
   */
  void sendI2CBytes(uint8_t address) const;

  /**
   * @brief Sends command data as ASCII over I2C.
   * 
   * @param address The I2C address to send to.
   */
  void sendI2CASCII(uint8_t address) const;

  /**
   * @brief Sends command data as raw bytes over UART.
   */
  void sendUartBytes(Stream& serial) const;

  /**
   * @brief Sends command data as ASCII over UART.
   */
  void sendUartASCII(Stream& serial) const;

  /**
   * @brief Reads command data as raw bytes from an I2C device.
   * 
   * @param address The I2C address to read from.
   */
  void readI2CBytes(uint8_t address);

  /**
   * @brief Reads command data as ASCII from an I2C device.
   * 
   * @param address The I2C address to read from.
   */
  void readI2CASCII(uint8_t address);

  /**
   * @brief Reads command data as raw bytes from UART.
   */
  void readUartBytes(Stream& serial);

  /**
   * @brief Reads command data as ASCII from UART.
   */
  void readUartASCII(Stream& serial);
};

#endif // TELEMETRY_COMMAND_HPP