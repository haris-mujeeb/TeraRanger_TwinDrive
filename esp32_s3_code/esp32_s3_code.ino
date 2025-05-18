#include <WiFi.h>
#include <HardwareSerial.h>
#include "comm.hpp"

// Serial Ports
HardwareSerial sensorSerial(1);   // Use UART1
HardwareSerial robotSerial(2);    // Use UART2

// WiFi Configuration
const char* SSID = "ESP32-Control";
const char* PASSWORD = "controlpass";
const char* SERVER_IP = "192.168.4.2";  // Python server IP
const uint16_t SERVER_PORT = 12345;     // Python server port

// ESP32 Server
WiFiServer espServer(SERVER_PORT);
WiFiClient pcClient;

// Data Structures
commandPacket cmdData;
telemetryPacket teleData;

// Function Prototypes
void setupWiFi();
void handleClientRequests();
void processCommand(const String& command);
void sendDataToPC(const String& data);
void forwardSensorData();

void setup() {
    Serial.begin(115200);
    sensorSerial.begin(115200, SERIAL_8N1, 18, 17);  // RX, TX for sensor
    robotSerial.begin(115200, SERIAL_8N1, 16, 15);     // RX, TX for robot

    setupWiFi();
}

void loop() {
    sendToFData();
    delay(10); // Adjust loop frequency as needed
    sendRobotData();
    delay(10); // Adjust loop frequency as needed
    handleClientRequests();
    delay(10); // Adjust loop frequency as needed
}

// Initialize ESP32 as an Access Point
void setupWiFi() {
    WiFi.softAP(SSID, PASSWORD);
    DEBUG_PRINT(DEBUG_COMM, "Access Point started");
    DEBUG_PRINT(DEBUG_COMM, "ESP32 IP Address: " + WiFi.softAPIP().toString());
    espServer.begin();
}

// Send data to the PC server
void sendDataToPC(const String& data) {
    if (pcClient.connect(SERVER_IP, SERVER_PORT)) {
        pcClient.print(data);
        DEBUG_PRINT(DEBUG_COMM, "Sent to PC: " + data);
    } else {
        DEBUG_PRINT(DEBUG_COMM, "Failed to connect to PC server!");
    }
    pcClient.stop();  // Close the connection
}

// Handle incoming commands from the PC
void handleClientRequests() {
    WiFiClient client = espServer.available();

    if (client) {
        DEBUG_PRINT(DEBUG_COMM, "Client connected");
        String clientMessage = "";

        while (client.connected() || client.available()) {
            if (client.available()) {
                char c = client.read();
                clientMessage += c;
            }
        }

        if (!clientMessage.isEmpty()) {
            DEBUG_PRINT(DEBUG_COMM, "Received from PC: " + clientMessage);
            processCommand(clientMessage);
        }

        client.stop();
        DEBUG_PRINT(DEBUG_COMM, "Client disconnected");
    }
}

// Parse and process incoming commands
void processCommand(const String& commandStr) {
    String command = commandStr;
    command.trim();

    int comma1 = command.indexOf(',');
    int comma2 = command.indexOf(',', comma1 + 1);

    if (comma1 == -1 || comma2 == -1) {
        DEBUG_PRINT(DEBUG_COMM, "Invalid command format");
        return;
    }

    String cmd = command.substring(0, comma1);
    cmd.trim();
    int value = command.substring(comma1 + 1, comma2).toInt();
    int speed = command.substring(comma2 + 1).toInt();

    if (cmd == "TURN") {
        cmdData.command = TumblerCommand::Rotate;
    } else if (cmd == "MOVE") {
        cmdData.command = TumblerCommand::Move;
    } else if (cmd == "STOP") {
        cmdData.command = TumblerCommand::Stop;
    }else {
        DEBUG_PRINT(DEBUG_COMM, "Unknown command: " + cmd);
        return;
    }

    cmdData.commandValue = value;
    cmdData.commandSpeed = speed;
    cmdData.sendUartASCII(robotSerial);
    DEBUG_PRINT(DEBUG_COMM, "Processed command: " + cmd + " Value: " + String(value) + " Speed: " + String(speed));
}

// Read sensor data and send to PC
void sendToFData() {  
  if (sensorSerial.available()) {
      String tofData = sensorSerial.readStringUntil('\n');
      String data = tofData + "\n";
      sendDataToPC(data);
      DEBUG_PRINT(DEBUG_COMM, "ToF Data Sent: " + data);
  }
}

// Read sensor data and send to PC
void sendRobotData() {  
  if (robotSerial.available()) {
    teleData.readUartASCII(robotSerial);
    String data = String(teleData.robotYawDegrees) + "," +
                  String(teleData.robotDistanceCm) + "," +
                  String(teleData.ultrasonicDistanceCm) + "\n";
    sendDataToPC(data);
    DEBUG_PRINT(DEBUG_COMM, "Robot Data Sent: " + data);
  }
}
