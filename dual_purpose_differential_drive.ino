// Motor Pins and Sensor Pins
int motorR = 4;
int motorRpwm = 3;
int motorLpwm = 6;
int motorL = 7;
int IR[] = { A5, A4, A3, A2, A1 };
const int trigL = 10, echoL = 9;
const int trigR = 11, echoR = 12;

// Line following error constants
float Kp = 13.0;
float Kd = 25.0;

// Tunnel navigation error constants
float Wd = 0.9;
float Wp = 1.3;

// Yellow DC motors constants if they have varying speed for the same PWM value
int Rd = 0;
int Ld = 0;

// Control Variables
int baseLine = 590;
int baseSpeed = 120;
int lastError = 0;
int last_wallError= 0;

void setup() {
  for (int i=0; i<5; i++) {
    pinMode(IR[i], INPUT);
  }
  Serial.begin(9600);
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  pinMode(trigR, OUTPUT);
  pinMode(echoR, INPUT);
}

void loop() {
  int error = get_error();
  
  // For debugging error based on sensor position
  Serial.print("Pos:");
  Serial.print(error);
  Serial.print(" ");

  if (error == 100) {
    int distL = getDistance(trigL, echoL);
    int distR = getDistance(trigR, echoR);

    // If no line & no walls detected (distance is 0)
    if (distL == 0 && distR == 0) {
      moveMotors(0, 0);
      return;  // Exit loop until next cycle
    }

    // Tunnel navigation algorithm using PD

    int wallError = distR - distL;
    int net_wallError = wallError - last_wallError;
    int wallAdjust = (wallError * Wp) + (Wd * net_wallError);
    last_wallError = wallError;
    
    // Individual motor speeds
    int right_turn = (baseSpeed + Rd) - wallAdjust;
    int left_turn = (baseSpeed + Ld) + wallAdjust;

    moveMotors(left_turn, right_turn);
    return;  // Exit loop so we don't run line following algorithm
  }

  // Line following algorithm
  int net_error = error - lastError;
  int turning_Speed = (Kp * error) + (Kd * net_error);
  lastError = error;

  // Individual motor speeds
  int leftSpeed = (baseSpeed + Ld) - turning_Speed;
  int rightSpeed = (baseSpeed + Rd) + turning_Speed;

  moveMotors(leftSpeed, rightSpeed);
}

// Line follower error function
int get_error() {
  int s0 = analogRead(IR[0]) >= baseLine;
  int s1 = analogRead(IR[1]) >= baseLine;
  int s2 = analogRead(IR[2]) >= baseLine;
  int s3 = analogRead(IR[3]) >= baseLine;
  int s4 = analogRead(IR[4]) >= baseLine;

  if (s0 && s1 && s2 && s3 && s4) return 100;  // Absence of blackline

  int sensor_position = s0*-2 + s1*-1 + s2*0 + s3*1 + s4*2;
  return sensor_position;
}

// Ultrasonic distance in cm
int getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 20000);
  float distance = (duration * 0.034) / 2;

  Serial.print("d: ");
  Serial.print(distance);
  Serial.print(" ");

  if (duration == 0) return 0.00;
  return distance;
}

// Motor speeds function
void moveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(motorL, 0);
  analogWrite(motorLpwm, leftSpeed);
  digitalWrite(motorR, 0);
  analogWrite(motorRpwm, rightSpeed);
  
  //Debugging individual motor speed
  Serial.print("VL:");
  Serial.print(leftSpeed);
  Serial.print(" ");
  Serial.print("VR:");
  Serial.println(rightSpeed);
}


// Created by members of Intro to Engineering, Team Volts @ Ashesi University, C2029
