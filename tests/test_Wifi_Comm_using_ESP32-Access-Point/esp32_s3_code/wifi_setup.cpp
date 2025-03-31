// wifi_setup.cpp
#include "Arduino.h"
#include "wifi_setup.h"
#include "comm.hpp"

const int EEPROM_SSID_ADDR = 0;
const int EEPROM_PASS_ADDR = 64; // Assuming SSID is max 63 chars
const int EEPROM_IP_ADDR = 128; // Assuming IP is max 15 chars
const int EEPROM_PORT_ADDR = 144;

char ssid[64] = "realme 9 Pro+";
char pass[64] = "muhammad";
// char serverIP[16] = "192.168.160.83";
uint16_t serverPort = 12345;

WiFiClient client;
WiFiServer server(serverPort);  // Create a server that listens on port 12345

telemetryPacket teleData;
commandPacket cmdData;

void setupWiFi() {

  // // *** Save Credentials ***
  // loadWifiCredentials();
  // loadServerCredentials();
  
  // if (strlen(ssid) == 0 || strlen(pass) == 0) {
  //   getWiFiCredentials();
  //   saveWiFiCredentials();
  // }

  // if (strlen(serverIP) == 0){
  //   getServerCredentials();
  //   saveServerCredentials();
  // }

  connectWiFi();

  // connectToServer();

  server.begin();
}

void getWiFiCredentials() {
  Serial.println("Enter WiFi SSID:");
  while (Serial.available() == 0);
  Serial.readBytesUntil('\n', ssid, sizeof(ssid) - 1);
  ssid[strlen(ssid)] = '\0';

  Serial.print("SSID: ");
  Serial.println(ssid);

  Serial.println("Enter WiFi Password:");
  while (Serial.available() == 0);
  Serial.readBytesUntil('\n', pass, sizeof(pass) - 1);
  pass[strlen(pass)] = '\0';

  Serial.print("Password: ");
  Serial.println("********");
}

void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n*** Connected to WiFi ***");
  printWiFiStatus();
}

void checkConnection() {
    if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();
  }
}

void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  Serial.print("Robot is hosting at -> ");
  Serial.print(WiFi.localIP()); Serial.print(":"); Serial.println(serverPort);
}

void loadWifiCredentials() {
  for (int i = 0; i < sizeof(ssid) - 1; i++) {
      ssid[i] = EEPROM.read(EEPROM_SSID_ADDR + i);
      if (ssid[i] == '\0') break;
  }
  ssid[sizeof(ssid) - 1] = '\0';

  for (int i = 0; i < sizeof(pass) - 1; i++) {
      pass[i] = EEPROM.read(EEPROM_PASS_ADDR + i);
      if (pass[i] == '\0') break;
  }
  pass[sizeof(pass) - 1] = '\0';

  if(strlen(ssid) > 0){
      Serial.print("Loaded SSID: ");
      Serial.println(ssid);
      Serial.print("Loaded Password: ");
      Serial.println("********");
  }
}

void saveWiFiCredentials() {
  for (int i = 0; i < strlen(ssid); i++) {
    EEPROM.write(EEPROM_SSID_ADDR + i, ssid[i]);
  }
  EEPROM.write(EEPROM_SSID_ADDR + strlen(ssid), '\0');

  for (int i = 0; i < strlen(pass); i++) {
    EEPROM.write(EEPROM_PASS_ADDR + i, pass[i]);
  }
  EEPROM.write(EEPROM_PASS_ADDR + strlen(pass), '\0');

  // EEPROM.commit(); // required for ESPXX
  Serial.println("WiFi credentials saved to EEPROM.");
}

void loadServerCredentials() {
    for (int i = 0; i < sizeof(serverIP) - 1; i++) {
        serverIP[i] = EEPROM.read(EEPROM_IP_ADDR + i);
        if (serverIP[i] == '\0') break;
    }
    serverIP[sizeof(serverIP) - 1] = '\0';
    serverPort = (EEPROM.read(EEPROM_PORT_ADDR) << 8) | EEPROM.read(EEPROM_PORT_ADDR + 1);

    if(strlen(serverIP) > 0){
        Serial.print("Loaded Server IP: ");
        Serial.println(serverIP);
        Serial.print("Loaded Server Port: ");
        Serial.println(serverPort);
    }
}

void saveServerCredentials() {
  for (int i = 0; i < strlen(serverIP); i++) {
      EEPROM.write(EEPROM_IP_ADDR + i, serverIP[i]);
  }
  EEPROM.write(EEPROM_IP_ADDR + strlen(serverIP), '\0');

  EEPROM.write(EEPROM_PORT_ADDR, (serverPort >> 8) & 0xFF);
  EEPROM.write(EEPROM_PORT_ADDR + 1, serverPort & 0xFF);

  // EEPROM.commit(); // required for ESPXX
  Serial.println("Server credentials saved to EEPROM.");
}

void getServerCredentials() {
    Serial.println("Enter Server IP:");
    while (Serial.available() == 0);
    Serial.readBytesUntil('\n', serverIP, sizeof(serverIP) - 1);
    serverIP[strlen(serverIP)] = '\0';

    Serial.println("Enter Server Port:");
    while (Serial.available() == 0);
    serverPort = Serial.parseInt();
    Serial.read(); // Consume the newline character
} 

void connectToServer() {
  if (client.connect(serverIP, serverPort)) {
    Serial.println("Connected to server");
  } else {
    Serial.println("Connection to server failed");
  }
}

void forwardTelemetryData(Stream& serial) {
  checkConnection();

  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New client connected !");
    } else {
      Serial.println("No device connected");
      return;
    }
  }

  static unsigned long lastTime = 0;
  if ( (client && client.connected()) && (millis() - lastTime > 10)) {
    lastTime = millis();
    
    teleData.readUartASCII(serial);

    String data = String(teleData.robotYawDegrees) + "," + String(teleData.robotDistanceCm) + "," + String(teleData.ultrasonicDistanceCm) + "\n";
      
    client.print(data);   // Send data to the Python server when requested

    Serial.print("Sent data to Python server: ");
    Serial.println(data);
  }
}

void readCommand(Stream& serial) {
  if (client.available()) {
    String command = client.readStringUntil('\n');  // Read command
    command.trim();
    Serial.print("Received command: ");
    Serial.println(command);

    if (command.startsWith("TURN")) {
      // Extract turn angle from command
      String turnValueStr = command.substring(4);
      turnValueStr.trim();
      if (turnValueStr.length() > 0) {
        cmdData.command = TumblerCommand::Rotate;
        cmdData.commandValue = turnValueStr.toInt();
        cmdData.sendUartASCII(serial);
        
        Serial.print("Turn command received: ");
        Serial.println(cmdData.commandValue);
      } else {
        Serial.println("Invalid TURN command: missing value");
        client.println("ERROR: Invalid TURN command");
      }

    } else if (command.startsWith("MOVE")) {
      // Parse the move command and execute it
      String moveValueStr = command.substring(4);
      moveValueStr.trim();
      if (moveValueStr.length() > 0) {
        cmdData.command = TumblerCommand::Move;
        cmdData.commandValue = moveValueStr.toInt();
        // cmdData.commandSpeed = moveSpeedStr.toInt();
        // cmdData.sendI2CBytes(SLAVE_ADDR);
        cmdData.sendUartASCII(serial);

        Serial.print("Move command recieved: ");
        Serial.println(cmdData.commandValue);
      } else {
        Serial.println("Invalid MOVE command: missing move value");
        client.println("ERROR: Invalid MOVE command");
      }
    } else {
      Serial.println("Unknown command");
      // client.stop();
      // Serial.println("Client disconnected.");
    }
  }
}