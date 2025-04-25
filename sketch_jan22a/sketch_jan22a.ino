//#include <Arduino.h>
//#include <AccelStepper.h>
//#include <MultiStepper.h>
//#include <Servo.h>
//#include <Wire.h>
//
//const int stepsPerRevolution = 200;
//
//// --- Pin Definitions (Original) ---
//#define X_STEP_PIN 54
//#define X_DIR_PIN 55
//#define X_ENABLE_PIN 38
//#define X_MIN_PIN 3
//
//#define Y_STEP_PIN 60
//#define Y_DIR_PIN 61
//#define Y_ENABLE_PIN 56
//#define Y_MIN_PIN 14
//
//#define E0_STEP_PIN 26
//#define E0_DIR_PIN 28
//#define E0_ENABLE_PIN 24
//
//#define Z_STEP_PIN 46
//#define Z_DIR_PIN 48
//#define Z_ENABLE_PIN 62
//
//#define motorInterfaceType 1
//
//// --- Stepper Instances (Original) ---
//AccelStepper stepperX(1, X_STEP_PIN, X_DIR_PIN);
//AccelStepper stepperY(1, Y_STEP_PIN, Y_DIR_PIN);
//AccelStepper stepperE0(1, E0_STEP_PIN, E0_DIR_PIN);
//AccelStepper stepperZ( motorInterfaceType , Z_STEP_PIN, Z_DIR_PIN);
//
//// --- Servo Instances (Original) ---
//Servo myServo1;
//Servo myServo2;
//Servo myServo3;
//
//// --- Variables (Original) ---
//long targetPositions[4] = {0, 0, 0, 0};
//int targetSpeeds[4] = {1000, 1000, 1000, 1000};
//int servoTargetAngles[3] = {0, 0, 0};
//int servoCurrentAngles[3] = {0, 0, 0};
//int encoderSpeed = 50;
//unsigned long lastEncoderReadTime = 0;
//int encoderAngles[4] = {0, 0, 0, 0};
//bool stopMotors[4] = {false, false, false, false};
//
//// --- NEW Constant for Servo Delay ---
//const int SERVO_MOVE_DELAY_MS = 500; // Milliseconds to wait for servo movement. Adjust if needed.
//
//void setup() {
//    // --- Stepper Setup (Original) ---
//    stepperX.setMaxSpeed(800);
//    stepperX.setAcceleration(500);
//    stepperY.setMaxSpeed(800);
//    stepperY.setAcceleration(500);
//    stepperE0.setMaxSpeed(800);
//    stepperE0.setAcceleration(500);
//    stepperZ.setMaxSpeed(800);
//    stepperZ.setAcceleration(500);
//
//    pinMode(X_ENABLE_PIN, OUTPUT);
//    pinMode(Y_ENABLE_PIN, OUTPUT);
//    pinMode(E0_ENABLE_PIN, OUTPUT);
//    pinMode(Z_ENABLE_PIN, OUTPUT);
//    // NOTE: Original code didn't explicitly set enable pins HIGH/LOW here. Keeping it that way.
//
//    // --- MODIFIED Servo Initialization ---
//    // Initialize ONLY Servo 1 to hold position continuously
//    myServo1.attach(4);
//
//    // Servos 2 and 3 remain detached until explicitly commanded to move.
//    // They will not hold any position initially.
//
//    // Initialize serial communication (Original)
//    Serial.begin(9600);
//
//    // Initialize servo current angles (MODIFIED: only Servo 1 relevant initially)
//    servoCurrentAngles[0] = myServo1.read(); // Read position of attached servo
//    servoCurrentAngles[1] = -1; // Indicate Servo 2 is detached
//    servoCurrentAngles[2] = -1; // Indicate Servo 3 is detached
//
//    Serial.println("Setup complete. Servo 1 active. Servos 2 & 3 detached.");
//}
//
//// --- Stepper Move Functions (Original - UNCHANGED) ---
//void moveX(int steps) {
//  stepperX.moveTo(steps); // Set target position
//  // NOTE: Original code enables/disables steppers INSIDE the move function loop usually,
//  // or relies on enable pin being LOW. Keeping original blocking logic.
//  while (stepperX.distanceToGo() != 0) { // Loop until target is reached
//    stepperX.run(); // Move one step (or manage acceleration)
//  }
//}
//
//void moveY(int steps) {
//  stepperY.moveTo(steps); // Set target position
//  while (stepperY.distanceToGo() != 0) { // Loop until target is reached
//    stepperY.run(); // Move one step
//  }
//}
//
//void moveE0(int steps) {
//  stepperE0.moveTo(steps); // Set target position
//  while (stepperE0.distanceToGo() != 0) { // Loop until target is reached
//    stepperE0.run(); // Move one step
//  }
//}
//
//void moveZ(int steps) {
//  stepperZ.moveTo(steps); // Set target position
//  while (stepperZ.distanceToGo() != 0) { // Loop until target is reached
//    stepperZ.run(); // Move one step
//  }
//}
//
//
//// --- MODIFIED moveServos Function ---
//void moveServos(int ServoIndex, int Servoangle) {
//    // Check if the angle is valid (Original check)
//    if(Servoangle >= 0 && Servoangle <= 180){
//        switch (ServoIndex) {
//            case 0: // Servo 1: Stays attached, just write new angle
//                // Optional safety: ensure it's attached
//                if (!myServo1.attached()) {
//                    myServo1.attach(4);
//                    delay(10); // Small delay if reattaching
//                }
//                myServo1.write(Servoangle);
//                servoCurrentAngles[0] = Servoangle; // Update known position
//                Serial.println("Servo 1 moving/holding.");
//                break;
//            case 1: // Servo 2: Attach, write, wait, detach
//                Serial.print("Servo 2 moving to "); Serial.print(Servoangle); Serial.println("...");
//                myServo2.attach(5);
//                delay(10); // Short delay for attach to stabilize
//                myServo2.write(Servoangle);
//                servoCurrentAngles[1] = Servoangle; // Store intended angle
//                delay(SERVO_MOVE_DELAY_MS); // *** WAIT FOR MOVEMENT ***
//                myServo2.detach();
//                servoCurrentAngles[1] = -1; // Mark as detached
//                Serial.println("Servo 2 moved and detached.");
//                break;
//            case 2: // Servo 3: Attach, write, wait, detach
//                Serial.print("Servo 3 moving to "); Serial.print(Servoangle); Serial.println("...");
//                myServo3.attach(6);
//                delay(10); // Short delay for attach to stabilize
//                myServo3.write(Servoangle);
//                servoCurrentAngles[2] = Servoangle; // Store intended angle
//                delay(SERVO_MOVE_DELAY_MS); // *** WAIT FOR MOVEMENT ***
//                myServo3.detach();
//                servoCurrentAngles[2] = -1; // Mark as detached
//                 Serial.println("Servo 3 moved and detached.");
//                break;
//        }
//    } else {
//      // Original error message format
//      Serial.print("only values 0-180");
//    }
//}
//
//// --- serialEvent Function (Original - UNCHANGED) ---
//// It correctly calls the modified moveServos function now.
//void serialEvent() {
//  if (Serial.available() > 0) {
//    String command = Serial.readStringUntil('\n');
//    command.trim();
//
//
//    if (command.startsWith("setcurrentpositionX")) {
//          int Position = command.substring(19).toInt();
//          stepperX.setCurrentPosition(Position);
//        } else if (command.startsWith("setcurrentpositionY")) {
//          int Position = command.substring(19).toInt();
//          stepperY.setCurrentPosition(Position);
//        } else if (command.startsWith("setcurrentpositionE0")) {
//          int Position = command.substring(20).toInt();
//          stepperE0.setCurrentPosition(Position);
//        } else if (command.startsWith("setcurrentpositionZ")) {
//          int Position = command.substring(19).toInt();
//          stepperZ.setCurrentPosition(Position);
//        } else
//        // --- Movement Commands ---
//        if (command.startsWith("moveX")) {
//          int steps = command.substring(5).toInt();
//          moveX(steps);
//        } else if (command.startsWith("moveY")) {
//          int steps = command.substring(5).toInt();
//          moveY(steps);
//        }
//        else if (command.startsWith("moveE0")) {
//            int steps = command.substring(6).toInt();
//            moveE0(steps);
//        }
//        else if (command.startsWith("moveZ")) {
//            int steps = command.substring(5).toInt();
//            moveZ(steps);
//        }
//        // --- Servo Commands ---
//        else if (command.startsWith("servo1")) {
//            int angle = command.substring(6).toInt();
//            moveServos(0, angle);
//        } else if (command.startsWith("servo2")) {
//            int angle = command.substring(6).toInt();
//            moveServos(1, angle);
//        } else if (command.startsWith("servo3")) {
//            int angle = command.substring(6).toInt();
//            moveServos(2, angle);
//        }
//        // --- Invalid Command ---
//        else {
//            Serial.println("ERROR");
//        }
//    }
//}
//
//// --- loop Function (Original - UNCHANGED) ---
//void loop() {
//  // Original loop was empty, serialEvent handles commands.
//}
