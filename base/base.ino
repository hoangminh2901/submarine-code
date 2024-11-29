#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebPage.h>

#define ss 15       // Chip Select
#define rst 16      // Reset
#define dio0 17      // IRQ

// WiFi credentials
const char *ssid = "Submarine go ọc ọc";
const char *password = "1234567890";

// Define HSPI pins
#define HSPI_SCK 14     // HSPI Clock
#define HSPI_MISO 12    // HSPI MISO
#define HSPI_MOSI 13    // HSPI MOSI

// Create an SPIClass instance for HSPI
SPIClass hspi(HSPI);

// LoRa frequency
#define LORA_FREQUENCY 433E6
#define SECURITY_PHRASE "MengVAnhNadine"  // Security phrase to identify messages

IPAddress local_ip(192, 168, 43, 120);
IPAddress gateway(192, 168, 43, 120);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);
String lastCommand = "STOP"; // Store the last command to send when polled
bool commandAcknowledged = true; // Flag to check if command was acknowledged by the boat

// Timing variables
unsigned long lastCommandSentTime = 0;
const unsigned long commandInterval = 1000; // 2 seconds between command sends if not acknowledged
const unsigned long ackTimeout = 5000; // Timeout after 10 seconds if no acknowledgment received
unsigned long lastReceiveAttempt = 0;
const unsigned long receiveTimeout = 100;

void setup() {
  Serial.begin(115200);
  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, ss); // SCK, MISO, MOSI, SS
  // Initialize LoRa module
  LoRa.setPins(ss, rst, dio0);
  LoRa.setSPI(hspi);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa initialization failed!");
    while (true);
  }
  Serial.println("LoRa initialized");

  // Setup Wi-Fi and start web server
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  // Define web server endpoints
  server.on("/", handle_OnConnect);
  server.on("/forward", []() { setCommand("FORWARD"); });
  server.on("/backward", []() { setCommand("BACKWARD"); });
  server.on("/turnRight", []() { setCommand("TURN_RIGHT"); });
  server.on("/turnLeft", []() { setCommand("TURN_LEFT"); });
  server.on("/pumpIn", []() { setCommand("PUMP_IN"); });
  server.on("/pumpOut", []() { setCommand("PUMP_OUT"); });
  server.on("/stop", []() { setCommand("STOP"); });

  server.begin();
  Serial.println("Web server started");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient(); // Handle web server requests

  // Check if there’s an incoming LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    handleLoRaPacket(packetSize);
  }

  /*
  // Re-enter receive mode periodically in case it was interrupted
  if (millis() - lastReceiveAttempt > receiveTimeout) {
    LoRa.receive();  // Keep LoRa in receive mode
    delay(100);
    lastReceiveAttempt = millis();
  } */
}

// Web UI handler
void handle_OnConnect() {
  server.send(200, "text/html", PAGE_MAIN);
}

void setCommand(String command) {
  lastCommand = command;        // Store the last command from the web interface
  commandAcknowledged = false;  // Reset acknowledgment flag
  Serial.println("Command set to: " + command);
  server.send(200, "text/plain", "Command set to: " + command);
}

// Handle incoming LoRa packets
void handleLoRaPacket(int packetSize) {
  if (packetSize == 0) return;

  char receivedText[256] = {0};
  int index = 0;

  while (LoRa.available() && index < sizeof(receivedText) - 1) {
    receivedText[index++] = (char)LoRa.read();
  }
  receivedText[index] = '\0';

  //Serial.println("Received: " + String(receivedText)); // Debug output

  // Check if the request starts with SECURITY_PHRASE
  if (strncmp(receivedText, SECURITY_PHRASE, strlen(SECURITY_PHRASE)) == 0) {
    String message = String(receivedText + strlen(SECURITY_PHRASE));

    if (message == "REQUEST_COMMAND") {
      //Serial.println("Received REQUEST_COMMAND from boat");
      sendCommandToBoat(); // Send the current command to the boat in response
    } else if (message == "ACK") {
      //Serial.println("Acknowledgment received from boat.");
      commandAcknowledged = true; // Set flag when acknowledgment is received
    } else {
      Serial.println(message);//"Unknown message received");
    }
  } else {
    //Serial.println("Unauthorized message received");
    //Serial.println(receivedText);
  }
}

void sendCommandToBoat() {
  // Only send the command if it hasn't been acknowledged
  if (!commandAcknowledged) {
    LoRa.idle();  // Switch to idle before sending
    LoRa.beginPacket();
    LoRa.print(SECURITY_PHRASE); // Add security phrase
    LoRa.print(lastCommand);     // Send the current command
    LoRa.endPacket();
    Serial.println("Sent command to boat: " + lastCommand);

    LoRa.receive();  // Return to receive mode after sending
    delay(10);
  }
}

