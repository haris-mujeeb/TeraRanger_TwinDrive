#include "wifi_setup.h"

void setup() {
  Serial.begin(9600);
  setupWiFi();
}

void loop() {
  checkConnection();
  forwardTelemetryData();
}
