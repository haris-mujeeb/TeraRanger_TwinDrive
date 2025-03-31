#include <WiFi.h>

const char* ssid_esp = "ESP32-Control";       // ESP32 Access Point SSID
const char* password_esp = "controlpass";     // ESP32 Access Point Password

const char* server_ip = "192.168.4.2";        // Python server IP (change this as needed)
const uint16_t server_port = 12345;           // Python server port (choose a port number)

// ESP32 as a server for receiving data from the PC
WiFiServer esp_server(server_port);   // Server to listen for incoming connections from PC
WiFiClient pc_client;                 // Client object for the connection to the Python server

// Example control system data
float sensorValue = 0.0;
float actuatorValue = 0.0;

void setup() {
  Serial.begin(115200);
  
  WiFi.softAP(ssid_esp, password_esp);        // Start ESP32 as an access point
  Serial.println("Access Point started");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.softAPIP());            // Print ESP32 IP address
  
  esp_server.begin();
  delay(1000);
}

void loop() {
  
  sensorValue = random(0, 100) / 10.0;  // Simulate sensor value (0-10)
  actuatorValue = random(0, 100) / 10.0;  // Simulate actuator value (0-10)
  String data = String(sensorValue) + "," + String(actuatorValue) + "\n";
  sendDataOverWifi(data);  // Function to send random data to the Python server

  // Handle incoming client connections (receive data from PC)
  handleClientRequests();

  delay(1);  // Adjust delay for desired frequency of sending data
}

void sendDataOverWifi(String data) {
  if (pc_client.connect(server_ip, server_port)) {
    pc_client.print(data);  // Send the data to Python server
    
    Serial.print("Sent to Python server: ");
    Serial.println(data);  // Print the sent data for debugging
  } else {
    Serial.println("Failed to connect to Python server!");
  }
  pc_client.stop();  // Close the connection to the Python server
}

void handleClientRequests() {
  WiFiClient client = esp_server.available();
  
  if (client) {
    Serial.println("Client connected");
    String client_message = "";
    
    // Read the data sent from the PC
    while (client.connected() || client.available()) {
      if (client.available()) {
        char c = client.read();  // Read the incoming data
        client_message += c;      // Append the character to the message string
      }
    }

    if (client_message.length() > 0) {
      Serial.print("Received from PC: ");
      Serial.println(client_message);  // Print the received data from the PC

      // Example: Process the data received from the PC (if needed)
      // For example, if it's a command to control an actuator:
      if (client_message.indexOf("SET_ACTUATOR=") >= 0) {
        String actuatorStr = client_message.substring(client_message.indexOf("=") + 1);
        actuatorValue = actuatorStr.toFloat();
        Serial.print("Actuator set to: ");
        Serial.println(actuatorValue);
      }
    }
    client.stop();  // Close the connection to the PC after processing
    Serial.println("Client disconnected");
  }
}