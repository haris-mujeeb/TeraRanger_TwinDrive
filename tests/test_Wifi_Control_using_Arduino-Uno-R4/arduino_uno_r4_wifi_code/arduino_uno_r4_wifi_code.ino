#include "comm.hpp"
#include "wifi_setup.h"


void setup() {
  Wire.begin();
  Serial.begin(9600);
  setupWiFi();
}

void loop() {
  checkConnection();
  forwardTelemetryData();
  readCommand();
}