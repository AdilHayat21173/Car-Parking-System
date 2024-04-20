#include <SoftwareSerial.h>

// L298N Motor Driver Pins

int S_A = 9;  // speed motor a
const int leftMotorForward = 2;
const int leftMotorBackward = 3;
const int rightMotorForward = 4;
const int rightMotorBackward = 5;
int S_B = 6;  // speed motor b
// Bluetooth Module Pins
const int bluetoothRx = 11;
const int bluetoothTx = 12;  // Corrected variable name
SoftwareSerial bluetooth(bluetoothRx, bluetoothTx);

void setup() {
  // Set motor pins as outputs
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  analogWrite(S_A, 90); // Adjust this value to decrease Motor A speed
  analogWrite(S_B, 90); 
  // Set up serial communication with Bluetooth module
  Serial.begin(9600);
  bluetooth.begin(9600);
}

void loop() {
  if (bluetooth.available() > 0) {
    char command = bluetooth.read();  // Added parentheses
    handleBluetoothCommand(command);
  }
}

void handleBluetoothCommand(char command) {
  switch (command) {
    case 'F':
      moveForward();
      break;
    case 'B':
      moveBackward();
      break;
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
    case 'S':
      stopMotors();
      break;
    default:
      // Ignore unrecognized commands
      break;
  }
}

void moveForward() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void moveBackward() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void turnLeft() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void turnRight() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void stopMotors() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

