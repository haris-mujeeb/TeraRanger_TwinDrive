#include <WiFi.h>
#include <WiFiUdp.h> // Include for UDP communication
#include <HardwareSerial.h>
#include "comm.hpp" // Assuming this header defines TumblerCommand, commandPacket, and telemetryPacket

// Serial Ports
HardwareSerial sensorSerial(1); // Use UART1 for sensor
HardwareSerial robotSerial(2);  // Use UART2 for robot

// --- WiFi Configuration ---
const char* SSID = "ESP32-Control";
const char* PASSWORD = "controlpass";

// --- UDP Configuration ---
// ESP32 IP address when acting as an Access Point (AP)
// Python client will connect to this network and typically get 192.168.4.2
const IPAddress localIP(192, 168, 4, 1);
const IPAddress gateway(192, 168, 4, 1);
const IPAddress subnet(255, 255, 255, 0);

const unsigned int ESP_LISTEN_PORT = 12345; // Port the ESP will listen on for commands from Python
const unsigned int PYTHON_LISTEN_PORT = 12346; // Port on Python side to send telemetry data to (Python's listening port)

// Python client's IP address. This will be updated once Python sends its first command.
// For initial testing, you might hardcode it if you know your Python machine's IP
// on the ESP's AP network, e.g., IPAddress(192, 168, 4, 2);
IPAddress pythonClientIP;

WiFiUDP Udp; // UDP instance

// Data Structures (from comm.hpp)
commandPacket cmdData;
telemetryPacket teleData;

// --- Timing for Data Sending ---
unsigned long previousMillis = 0;
const long interval = 10; // Send data every 10 milliseconds

// --- Debugging ---
// Assuming DEBUG_COMM and DEBUG_PRINT are defined in comm.hpp or elsewhere
#ifndef DEBUG_COMM
#define DEBUG_COMM true // Define if not already defined
#endif
#ifndef DEBUG_PRINT
#define DEBUG_PRINT(flag, msg) do { if (flag) { Serial.println(msg); } } while (0)
#endif


// Function Prototypes
void setupWiFi();
void handleUdpCommands(); // Refactored from handleClientRequests
void processCommand(const String& command);
void sendTelemetryToPC(); // Refactored from sendDataToPC, integrated into forwardSensorData for 10ms loop
void forwardSensorData();

void setup() {
    Serial.begin(115200);
    sensorSerial.begin(115200, SERIAL_8N1, 18, 17); // RX, TX for sensor
    robotSerial.begin(115200, SERIAL_8N1, 16, 15);   // RX, TX for robot

    setupWiFi();

    // Initialize pythonClientIP to an invalid address initially
    pythonClientIP = IPAddress(0, 0, 0, 0);
}

void loop() {
    // Process incoming UDP commands from Python
    handleUdpCommands();

    // Read sensor data and attempt to send every 10ms
    forwardSensorData();
}

// Initialize ESP32 as an Access Point and start UDP listener
void setupWiFi() {
    WiFi.softAPConfig(localIP, gateway, subnet); // Set static IP for AP
    WiFi.softAP(SSID, PASSWORD);
    DEBUG_PRINT(DEBUG_COMM, "Access Point started");
    DEBUG_PRINT(DEBUG_COMM, "ESP32 IP Address: " + WiFi.softAPIP().toString());

    Udp.begin(ESP_LISTEN_PORT);
    DEBUG_PRINT(DEBUG_COMM, "Listening for UDP commands on port: " + String(ESP_LISTEN_PORT));
}

// Handle incoming UDP commands from the PC
void handleUdpCommands() {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        // Read the packet into a temporary buffer
        char incomingPacketBuffer[255]; // Max UDP packet size is often 255-512 bytes for simple messages
        int len = Udp.read(incomingPacketBuffer, sizeof(incomingPacketBuffer) - 1);
        if (len > 0) {
            incomingPacketBuffer[len] = 0; // Null-terminate the string
        }

        // Store the sender's IP. This is our Python client.
        // We only need the IP, as the Python script will listen on a known port (PYTHON_LISTEN_PORT)
        pythonClientIP = Udp.remoteIP();
        // unsigned int remotePort = Udp.remotePort(); // Not strictly needed for sending back to a known port

        String clientMessage = String(incomingPacketBuffer);
        DEBUG_PRINT(DEBUG_COMM, "Received command from " + pythonClientIP.toString() +
                                 " -> " + clientMessage);

        processCommand(clientMessage);
    }
}

// Parse and process incoming commands (no change needed for UDP)
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
    } else {
        DEBUG_PRINT(DEBUG_COMM, "Unknown command: " + cmd);
        return;
    }

    cmdData.commandValue = value;
    cmdData.commandSpeed = speed;
    cmdData.sendUartASCII(robotSerial); // Assuming this function exists and is correctly defined
    DEBUG_PRINT(DEBUG_COMM, "Processed command: " + cmd + " Value: " + String(value) + " Speed: " + String(speed));
}

// Read sensor data and prepare to send to PC via UDP
void forwardSensorData() {
    static String lastTofData = "";
    static String lastRobotData = "";
    
    if (sensorSerial.available()) {
        lastTofData = sensorSerial.readStringUntil('\n');
        DEBUG_PRINT(DEBUG_COMM, "Rec ToF: " + lastTofData);
    }

    if (robotSerial.available()) {
        lastRobotData = "RB\t" + robotSerial.readStringUntil('\n');
        DEBUG_PRINT(DEBUG_COMM, "Rec Robot: " + lastRobotData);
    }

    // teleData.readUartASCII(robotSerial);


    // Only send data if enough time has passed and we know the Python client's IP
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        if (pythonClientIP != IPAddress(0, 0, 0, 0)) { // Only send if Python client IP is known
            String combinedData = lastTofData + "\n" + lastRobotData;

            // Send telemetry data
            Udp.beginPacket(pythonClientIP, PYTHON_LISTEN_PORT);
            Udp.print(combinedData); // Use Udp.print() for String
            Udp.endPacket();

            DEBUG_PRINT(DEBUG_COMM, combinedData);
            // Commented out verbose debug print to avoid flooding serial at 10ms interval
        } else {
            DEBUG_PRINT(DEBUG_COMM, "Waiting for Python client to send first command...");
            // Commented out verbose debug print
        }
    }
}