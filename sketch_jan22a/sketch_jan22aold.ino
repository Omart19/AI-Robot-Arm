#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <Wire.h> 

const int stepsPerRevolution = 200;



// Define motor pins:
#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38
#define X_MIN_PIN 3

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56
#define Y_MIN_PIN 14

#define E0_STEP_PIN 26
#define E0_DIR_PIN 28
#define E0_ENABLE_PIN 24

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62

#define motorInterfaceType 1

AccelStepper stepperX(1, X_STEP_PIN, X_DIR_PIN);  // 1 = Easy Driver interface
AccelStepper stepperY(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperE0(1, E0_STEP_PIN, E0_DIR_PIN);  // 1 = Easy Driver interface
AccelStepper stepperZ( motorInterfaceType , Z_STEP_PIN, Z_DIR_PIN);

// Create servo instances
Servo myServo1;
Servo myServo2;
Servo myServo3;

// Variables to store target positions and speeds
long targetPositions[4] = {0, 0, 0, 0};  // For steppers (X, Y, E0, Z)
int targetSpeeds[4] = {1000, 1000, 1000, 1000};      // For steppers (set default speeds)
int servoTargetAngles[3] = {0, 0, 0};     // For servos (target angles)
int servoCurrentAngles[3] = {0, 0, 0};  // For servos (current angles)
int encoderSpeed = 50; // Default encoder reading speed (0-100)
unsigned long lastEncoderReadTime = 0; 
int encoderAngles[4] = {0, 0, 0, 0}; // Declare encoderAngles globally
bool stopMotors[4] = {false, false, false, false}; // Flags to signal motor stop



// void setup();
// void moveX(int steps);
// void moveY(int steps);
// void moveE0(int steps);
// void moveZ(int steps);
// void moveServos(int ServoIndex, int Servoangle);
// void serialEvent();
// void loop();
void setup() {

    
  stepperX.setMaxSpeed(800);
  stepperX.setAcceleration(500);
  stepperY.setMaxSpeed(800);
  stepperY.setAcceleration(500);
  stepperE0.setMaxSpeed(800);
  stepperE0.setAcceleration(500);
  stepperZ.setMaxSpeed(800);
  stepperZ.setAcceleration(500);

    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(Y_ENABLE_PIN, OUTPUT);
    pinMode(E0_ENABLE_PIN, OUTPUT);
    pinMode(Z_ENABLE_PIN, OUTPUT);


    // Initialize servos
    myServo1.attach(4); 
    myServo2.attach(5);
    myServo3.attach(6);


    // Initialize serial communication
    Serial.begin(9600);

    // Initialize servo current angles
    servoCurrentAngles[0] = myServo1.read(); 
    servoCurrentAngles[1] = myServo2.read();
    servoCurrentAngles[2] = myServo3.read();
}

void moveX(int steps) {
  stepperX.moveTo(steps); // Set target position
  while (stepperX.distanceToGo() != 0) { // Loop until target is reached
    stepperX.run(); // Move one step
  }
}

void moveY(int steps) {
  stepperY.moveTo(steps); // Set target position
  while (stepperY.distanceToGo() != 0) { // Loop until target is reached
    stepperY.run(); // Move one step
  }
}

void moveE0(int steps) {
  stepperE0.moveTo(steps); // Set target position
  while (stepperE0.distanceToGo() != 0) { // Loop until target is reached
    stepperE0.run(); // Move one step
  }
}

void moveZ(int steps) {
  stepperZ.moveTo(steps); // Set target position
  while (stepperZ.distanceToGo() != 0) { // Loop until target is reached
    stepperZ.run(); // Move one step
  }
}



void moveServos(int ServoIndex, int Servoangle) {
    // Check if any servo needs to move
    if(Servoangle >= 0 && Servoangle <= 180){
      switch (ServoIndex) {
        case 0: myServo1.write(Servoangle); break;
        case 1: myServo2.write(Servoangle); break;
        case 2: myServo3.write(Servoangle); break;
      }
        
    } else {
      Serial.print("only values 0-180");
    }
}

void serialEvent() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    
    if (command.startsWith("setcurrentpositionX")) {
          int Position = command.substring(19).toInt();
          stepperX.setCurrentPosition(Position);
        } else if (command.startsWith("setcurrentpositionY")) {
          int Position = command.substring(19).toInt();
          stepperY.setCurrentPosition(Position);
        } else if (command.startsWith("setcurrentpositionE0")) {
          int Position = command.substring(20).toInt();
          stepperE0.setCurrentPosition(Position);
        } else if (command.startsWith("setcurrentpositionZ")) {
          int Position = command.substring(19).toInt();
          stepperZ.setCurrentPosition(Position);
        } else
        // --- Movement Commands ---
        if (command.startsWith("moveX")) {
          int steps = command.substring(5).toInt();
          moveX(steps);
        } else if (command.startsWith("moveY")) {
          int steps = command.substring(5).toInt();
          moveY(steps);
        } 
        else if (command.startsWith("moveE0")) {
            int steps = command.substring(6).toInt();
            moveE0(steps);
        }
        else if (command.startsWith("moveZ")) {
            int steps = command.substring(5).toInt();
            moveZ(steps); 
        }
        // --- Servo Commands ---
        else if (command.startsWith("servo1")) {
            int angle = command.substring(6).toInt();
            moveServos(0, angle); 
        } else if (command.startsWith("servo2")) {
            int angle = command.substring(6).toInt();
            moveServos(1, angle); 
        } else if (command.startsWith("servo3")) {
            int angle = command.substring(6).toInt();
            moveServos(2, angle); 
        }
        // --- Invalid Command ---
        else {
            Serial.println("ERROR"); 
        }
    }
}

void loop() {

}
