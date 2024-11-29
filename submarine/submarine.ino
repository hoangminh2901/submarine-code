#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "esp_task_wdt.h" // Include ESP32 task watchdog library

#define I2C_SDA 21
#define I2C_SCL 22
#define ss 15   // LoRa Chip Select
#define rst 5   // LoRa Reset
#define dio0 27 // LoRa IRQ
// Define HSPI pins
#define HSPI_SCK 14  // HSPI Clock
#define HSPI_MISO 12 // HSPI MISO
#define HSPI_MOSI 13 // HSPI MOSI

// Define motor control pins
int M1_Left = 32;
int M1_Right = 33;
int M2_Left = 26;
int M2_Right = 25;
int M3_Left = 2;
int M3_Right = 4;
int M4_Left = 35;
int M4_Right = 34;

// Define pump control pins
#define pumpInPin 18
#define pumpOutPin 19

// Security phrase for command validation
#define SECURITY_PHRASE "MengVAnhNadine"

// LoRa frequency
#define LORA_FREQUENCY 433E6

TwoWire I2CBNO = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &I2CBNO);

// Create an SPIClass instance for HSPI
SPIClass hspi(HSPI);

// Define timing constants
const unsigned long COMMAND_POLL_INTERVAL = 2000; // Poll for commands every 5 seconds
const unsigned long DATA_TRANSMIT_INTERVAL = 150; // Transmit data every 1 second
const unsigned long MOTOR_RUN_LIMIT = 10000;      // Motor runs for 10 seconds max

unsigned long lastCommandPollTime = 0;
unsigned long lastDataTransmitTime = 0;
unsigned long motorStartTime = 0;
bool isMotorRunning = false;

// PID constants
const float Kp = 0.5;
const float Ki = 0.001;
const float Kd = 0.01;

float previousError = 0;
float integral = 0;
unsigned long lastTimePID = 0;
bool isPIDActive = false;
int leftSpeed = 0;
int rightSpeed = 0;

void setup()
{
  Serial.begin(115200);

  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, ss); // SCK, MISO, MOSI, SS
  // Initialize LoRa
  LoRa.setPins(ss, rst, dio0);
  LoRa.setSPI(hspi);
  while (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("LoRa initialization failed!");
    delay(1000);
  }
  Serial.println("LoRa initialized");

  // Initialize BNO055 for heading
  I2CBNO.begin(I2C_SDA, I2C_SCL);
  while (!bno.begin())
  {
    Serial.println("BNO055 initialization failed!");
    delay(1000);
  }

  pinMode(M1_Left, OUTPUT);
  pinMode(M1_Right, OUTPUT);
  pinMode(M2_Left, OUTPUT);
  pinMode(M2_Right, OUTPUT);
  pinMode(pumpInPin, OUTPUT);
  pinMode(pumpOutPin, OUTPUT);
}

void loop()
{
  unsigned long currentTime = millis();

  // 1. Data Collection & Transmission
  if (currentTime - lastDataTransmitTime >= DATA_TRANSMIT_INTERVAL)
  {
    collectAndSendData(); // Collect data from BNO055 and send to base station
    lastDataTransmitTime = currentTime;
  }

  // 2. Poll for Commands from Base Station
  if (currentTime - lastCommandPollTime >= COMMAND_POLL_INTERVAL)
  {
    delay(50);
    pollForCommands(); // Boat initiates command check with base station
    lastCommandPollTime = currentTime;
  }

  // 3. Motor Operation Check
  if (isMotorRunning && (currentTime - motorStartTime >= MOTOR_RUN_LIMIT))
  {
    handleStop();
    isMotorRunning = false;
    Serial.println("Motor stopped after running for 10 seconds");
  }
}

void collectAndSendData()
{
  sensors_event_t orientationData, angVelocityData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Get Absolute Orientation (Euler Angles)
  // bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
  // Get Angular Velocity Vector
  // bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Get Linear Acceleration Vector
  // bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
  unsigned long currentTime = millis();
  // Prepare data message
  char dataMessage[256]; // Increase buffer size for additional data
  snprintf(dataMessage, sizeof(dataMessage), "%sDATA,%lums,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
           SECURITY_PHRASE,
           currentTime,
           orientationData.orientation.x,   // Absolute orientation (X - Euler)
           orientationData.orientation.y,   // Absolute orientation (Y - Euler)
           orientationData.orientation.z,   // Absolute orientation (Z - Euler)
           angVelocityData.gyro.x,          // Angular velocity (X)
           angVelocityData.gyro.y,          // Angular velocity (Y)
           angVelocityData.gyro.z,          // Angular velocity (Z)
           linearAccelData.acceleration.x,  // Linear acceleration (X)
           linearAccelData.acceleration.y,  // Linear acceleration (Y)
           linearAccelData.acceleration.z); // Linear acceleration (Z));         // Temperature

  // Send data over LoRa
  LoRa.beginPacket();
  LoRa.print(dataMessage);
  LoRa.endPacket();
  delay(10);
  Serial.print("Sent data: ");
  Serial.println(dataMessage);
}

void pollForCommands()
{
  // Send REQUEST_COMMAND message
  // String requestMessage = String(SECURITY_PHRASE) + "REQUEST_COMMAND";
  char requestMessage[64] = {0}; // Make sure the size is sufficient for the combined message

  // Convert the String to a char array
  snprintf(requestMessage, sizeof(requestMessage), "%s%s", SECURITY_PHRASE, "REQUEST_COMMAND");
  LoRa.beginPacket();
  LoRa.print(requestMessage);
  LoRa.endPacket();
  // Serial.println("Polling for commands...: ");
  // Serial.print(requestMessage);
  // Serial.println();
  // delay(100);
  // LoRa.idle();  // Short delay to allow base station to respond
  LoRa.receive(); // Switch back to receive mode
  delay(100);
  // Check if there's a response
  int packetSize = LoRa.parsePacket();
  // if (packetSize == 0) return;

  char commandMessage[256] = {0};
  int index = 0;

  while (LoRa.available() && index < sizeof(commandMessage) - 1)
  {
    commandMessage[index++] = (char)LoRa.read();
  }
  commandMessage[index] = '\0';
  // Serial.println(commandMessage);
  handleCommand(commandMessage);
}

void handleCommand(const char *command)
{
  // Verify command with security phrase
  if (strncmp(command, SECURITY_PHRASE, strlen(SECURITY_PHRASE)) == 0)
  {
    String actualCommand = String(command + strlen(SECURITY_PHRASE));
    Serial.print("Received command: ");
    Serial.println(actualCommand);

    if (actualCommand == "FORWARD")
    {
      sendAcknowledgment();
      handleForward();
    }
    else if (actualCommand == "BACKWARD")
    {
      sendAcknowledgment();
      handleBackward();
    }
    else if (actualCommand == "TURN_RIGHT")
    {
      sendAcknowledgment();
      handleTurnRight();
    }
    else if (actualCommand == "TURN_LEFT")
    {
      sendAcknowledgment();
      handleTurnLeft();
    }
    else if (actualCommand == "PUMP_IN")
    {
      sendAcknowledgment();
      handlePumpIn();
    }
    else if (actualCommand == "PUMP_OUT")
    {
      sendAcknowledgment();
      handlePumpOut();
    }
    else if (actualCommand == "STOP")
    {
      handleStop();
      sendAcknowledgment();
    }
    else
    {
      Serial.println("Unknown command received");
    }
  }
  else
  {
    Serial.println("Invalid or unauthorized command received");
  }
}

void moveMotor1(int direction, int speed = 255)
{
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1)
  {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  analogWrite(M1_Left, inPin1 ? speed : 0);
  analogWrite(M1_Right, inPin2 ? speed : 0);
}
void moveMotor2(int direction, int speed = 255)
{
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1)
  {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  analogWrite(M2_Left, inPin1 ? speed : 0);
  analogWrite(M2_Right, inPin2 ? speed : 0);
  // digitalWrite(M2_Left, inPin1);
  // digitalWrite(M2_Right, inPin2);
}

void stopMotor1()
{ // Function to stop Motor 1
  analogWrite(M1_Left, 0);
  analogWrite(M1_Right, 0);
}

void stopMotor2()
{ // Function to stop Motor 2
  analogWrite(M2_Left, 0);
  analogWrite(M2_Right, 0);
}

void handleForward()
{
  moveMotor1(1);
  moveMotor2(1);
  Serial.println("Moving forward with data streaming enabled");
}

void handleBackward()
{
  moveMotor1(2);
  moveMotor2(2);
  Serial.println("Moving backward with data streaming enabled");
}

void handleTurnRight()
{
  moveMotor1(2);
  moveMotor2(1);
  Serial.println("Turning right with data streaming enabled");
}

void handleTurnLeft()
{
  moveMotor1(1);
  moveMotor2(2);
  Serial.println("Turning left with data streaming enabled");
}

void handlePumpIn()
{
  digitalWrite(pumpInPin, HIGH);
  digitalWrite(pumpOutPin, LOW);
  Serial.println("Pumping in with data streaming enabled");
}

void handlePumpOut()
{
  digitalWrite(pumpInPin, LOW);
  digitalWrite(pumpOutPin, HIGH);
  Serial.println("Pumping out with data streaming enabled");
}

void handleStop()
{
  stopMotor1();
  stopMotor2();
  Serial.println("Stopping all motors and data streaming");
}

void sendAcknowledgment()
{
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print(SECURITY_PHRASE); // Add security phrase for consistency
  LoRa.print("ACK");           // Send acknowledgment
  LoRa.endPacket();
  LoRa.receive(); // Return to receive mode after sending
}