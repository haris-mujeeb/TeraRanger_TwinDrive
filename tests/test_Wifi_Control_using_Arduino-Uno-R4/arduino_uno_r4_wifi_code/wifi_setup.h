// wifi_setup.h

#ifndef WIFI_SETUP_H
#define WIFI_SETUP_H

#include "WiFiS3.h"
#include <Arduino.h>
#include <EEPROM.h>

extern char ssid[64];
extern char pass[64];
extern char serverIP[16];
extern uint16_t serverPort;

void setupWiFi();
void getWiFiCredentials();
void connectWiFi();
void checkConnection();
void printWiFiStatus();
void loadWifiCredentials();
void saveWiFiCredentials();
void loadServerCredentials();
void saveServerCredentials();
void getServerCredentials();
void connectToServer();
void forwardTelemetryData();
void readCommand();


#endif // WIFI_SETUP_H