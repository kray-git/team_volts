#include <Servo.h>
#include <math.h>

// Pins
const int trigPin = 10;
const int echoPin = 9;

// Servos Configuration
const int servos = 4;
const int pins[servos] = { 3, 5, 6, 11 };  // Base, Shoulder, Elbow, Gripper
Servo servos[servos];
int currentPulse[servos] = { 500, 700, 1500, 800 }; // Home position

// Arm physical dimensions (cm)
const float L1 = 14.50;
const float L2 = 11.50;
const float L3 = 15.00;
const float sensor_Offset = 4.50;

// State Management to isolate various phases of operation
enum State { Scanning, Picking, Grabbing, Dropping };
State currentState = Scanning;

float targetDistance = 0;
int basePulseAtDetection = 500;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(pins[i]);
    servos[i].writeMicroseconds(currentPulse[i]);
  }

  Serial.println("System Initialized");
}

void moveServo(int index, int targetPulse) {
  targetPulse = constrain(targetPulse, 500, 2500);
  if (targetPulse > currentPulse[index]) {
    for (int p = currentPulse[index]; p <= targetPulse; p++) {
      servos[index].writeMicroseconds(p);
      delay(2);
    }
  } else {
    for (int p = currentPulse[index]; p >= targetPulse; p--) {
      servos[index].writeMicroseconds(p);
      delay(2);
    }
  }
  currentPulse[index] = targetPulse;
}

void loop() {
  printStatus();
  switch (currentState) {
    case Scanning:
      scanForObject();
      break;

    case Picking:
      reachObject(targetDistance);
      currentState = Grabbing;
      break;

    case Grabbing:
      Serial.println("Closing Gripper.");
      moveServo(3, 2100);  // Close Gripper
      delay(1000);
      currentState = Dropping;
      break;

    case Dropping:
      executeDrop();
      currentState = Scanning;
      break;
  }
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  float d = duration * 0.034 / 2;
  return (d == 0) ? 999 : d;
}

void scanForObject() {
  static int pulse = 900;
  static int step = 10; // Scanning speeding regulator

  servos[0].writeMicroseconds(pulse);
  currentPulse[0] = pulse;

  float d = getDistance();
  if (d > 5 && d < 22) {
    targetDistance = sensor_Offset + d;
    basePulseAtDetection = pulse;
    Serial.print("Target Found at: ");
    Serial.println(d);
    currentState = Picking;
  } else {
    pulse += step;
    if (pulse >= 2500 || pulse <= 500) step *= -1;
    delay(50);
  }
}

void reachObject(float r2) {
  float verticalDist = 6 ; // Vertical distance from object position to the end effector;
  float r1 = sqrt(sq(r2) + sq(verticalDist));

  float cosA = (sq(L2) + sq(L3) - sq(r1)) / (2 * L2 * L3);
  float elbowA = acos(constrain(cosA, -1, 1)) * 180 / PI;

  float psi = atan2(verticalDist, r2) * 180 / PI;
  float cosAlpha = (sq(L2) + sq(r1) - sq(L3)) / (2 * L2 * r1);
  float alpha = acos(constrain(cosAlpha, -1, 1)) * 180 / PI;
  float shoulderA = alpha + psi;

  // Linear equation assignments
  int elbowPulse = (750) + (12.937 * elbowA);
  int shoulderPulse = 1700 - (10 * shoulderA);


  Serial.print("elbowA: ");
  Serial.print(elbowA);
  Serial.print(" ");
  Serial.print("shoulderA: ");
  Serial.println(shoulderA);
  Serial.print("elbowPulse: ");
  Serial.print(elbowPulse);
  Serial.print(" ");
  Serial.print("shoulderPulse: ");
  Serial.println(shoulderPulse);

  moveServo(2, elbowPulse);    // Move Elbow
  delay(20);
  moveServo(1, shoulderPulse);  // Move Shoulder
  delay(500);
}

void executeDrop() {
  // Lift arm for safe travel
  moveServo(1, 600);
  moveServo(2, 1300);
  
  
  // Rotate to drop zone (90 degrees offset)
  int dropPulse = (basePulseAtDetection + 700);
  if (dropPulse > 2500) dropPulse -= 5000;

  moveServo(0, dropPulse);
  moveServo(3, 600);  // Open Gripper
  delay(500);

  // Reset to home position
  moveServo(1, 700);
  moveServo(2, 1500);
  moveServo(0, 500);
}

// For debugging currentState every half second
void printStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {  // Print every half second
    Serial.print("Current State: ");
    Serial.println(currentState);
    lastPrint = millis();
  }
}



// Created by members of Intro to Engineering, Team Volts @ Ashesi University, C2029
