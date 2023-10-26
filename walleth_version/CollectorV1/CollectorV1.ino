#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////////////////
// Set pin
#define R_S A0   // Infrared sensor Right
#define CR_S A1  // Infrared sensor Center Right
#define CL_S A2  // Infrared sensor Center Left
#define L_S A3   // Infrared sensor Left

// Set speed
#define SLOW_SPEED 45
//#define MEDIUM_SPEED 55
#define FAST_SPEED 55

int defaultSpeed = 100;
int stoptime = 0;
int can_counter = 0;

// Servo code
int min_pos = 60;
int max_pos = 180;
int pos = 0;

////////////////////////////////////////////////////////////////////////////////////////
// PID Controller

// Gain k=[0;1]
// Tuning parameter:
// Start with kp=1 and decrease value to stable system
//Then increase ki and kd if needed
double kp = 0.29;    // Proportional gain
double ki = 0.027;  // Integral gain
double kd = 0;    // Derivative gain *************************** possible delete

//Initialize PID components to 0
double P = 0;  // Proportional component of the control output
double I = 0;  // Integral component of the control output
double D = 0;  // Derivative component of the control output

int error;
int previousError = 0;
int smallError = defaultSpeed * 0.5;    // Need to calibrate this parameter
int mediumError = defaultSpeed * 1;  // Need to calibrate this parameter
int bigError = defaultSpeed * 2.2;     // Need to calibrate this parameter
bool stopped = false;


int lastError = 0;
int lastLineSignal = 0;

int leftSpeed = 0;
int rightSpeed = 0;


////////////////////////////////////////////////////////////////////////////////////////
//Functions

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Servo myservo1;
Servo myservo2;


  void take_trash() {
    for (pos = min_pos; pos <= max_pos; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  void release_trash() {
     for (pos = max_pos; pos >= min_pos; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo1.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }

  // void shake_trash() {
  //   return 0;
  // }

  void right_Track_to_Can() {
    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    motorR->setSpeed(50);
    motorL->setSpeed(50);
    delay(750);
    motorR->run(BACKWARD);
    motorL->run(FORWARD);
    delay(400);
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(300);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
  }
  void right_Can_to_Track() {
    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(300);
    motorR->run(FORWARD);
    motorL->run(BACKWARD);
    delay(400);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
  }

  void left_Track_to_Can() {
    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    motorR->setSpeed(50);
    motorL->setSpeed(50);
    delay(750);
    motorR->run(FORWARD);
    motorL->run(BACKWARD);
    delay(400);
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(300);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
  }

  void left_Can_to_Track() {
    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(300);
    motorR->run(BACKWARD);
    motorL->run(FORWARD);
    delay(400);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
  }

  void sprint() {
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(800);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
  }

// Function to get the error from the line-following sensors (0-2).
int getError() {
  // Read the sensor values (0 for dark, 1 for bright)
  int sensorL = digitalRead(L_S);
  int sensorCL = digitalRead(CL_S);
  int sensorCR = digitalRead(CR_S);
  int sensorR = digitalRead(R_S);


  Serial.print(sensorL);
  Serial.print(sensorCL);
  Serial.print(sensorCR);
  Serial.print(sensorR);
  Serial.println(" ");

  // Determine the error based on sensor readings
  if (sensorL == 0 && sensorCL == 1 && sensorCR == 1 && sensorR == 1) {
    Serial.println("bigError turnLeft");
    return -bigError;  // The robot is significantly to the right of the line
  } else if (sensorL == 0 && sensorCL == 0 && sensorCR == 1 && sensorR == 1) {
    Serial.println("mediumError turnLeft");
    return -mediumError;  // The robot is the right of the line
  } else if (sensorL == 0 && sensorCL == 0 && sensorCR == 0 && sensorR == 1) {
    Serial.println("smallError turnLeft");
    return -smallError;  // The robot is slightly to the right of the line
  } else if (sensorL == 1 && sensorCL == 0 && sensorCR == 0 && sensorR == 1) {
    Serial.println("noError");
    I = 0;
    return 0;  // The robot is on the line (centered)
  } else if (sensorL == 1 && sensorCL == 0 && sensorCR == 0 && sensorR == 0) {
    Serial.println("smallError turnRight");
    return smallError;  // The robot is slightly to the left of the line
  } else if (sensorL == 1 && sensorCL == 1 && sensorCR == 0 && sensorR == 0) {
    Serial.println("mediumError turnRight");
    return mediumError;  // The robot is the left of the line
  } else if (sensorL == 1 && sensorCL == 1 && sensorCR == 1 && sensorR == 0) {
    Serial.println("bigError turnRight");
    return bigError;  // The robot is significantly to the left of the line
  } else if (sensorL == 0 && sensorCL == 0 && sensorCR == 0 && sensorR == 0) {
    return 1;                                                                                                                                                                                                                  // Return obstacle 1
  } else if ((sensorL == 0 && sensorCL == 1 && sensorCR == 1 && sensorR == 0) || (sensorL == 0 && sensorCL == 1 && sensorCR == 0 && sensorR == 0) || (sensorL == 0 && sensorCL == 0 && sensorCR == 1 && sensorR == 0)) {  // Special case: Only the center sensor detects a bright surface (possible obstacle)
    return 2;                                                                                                                                                                                                                  // Return obstacle 2
  } else if (sensorL == 1 && sensorCL == 1 && sensorCR == 1 && sensorR == 1) {                                                                                                                                                 // Special case: All sensors detect a white surface (end of track)
    return 3;
  }
  return 0;  // Default case: Unknown/error state. Proceed with line tracking
}

void PID_line_tracking() {
  //stopped = false;

  // PID control
  // Proportional component of the control output
  P = error;
  // Integral component of the control output
  I += error;
  // Derivative component of the control output
  D = error - previousError;

  // Update the previous error for the next iteration
  previousError = error;

  // Combine all components to get the PID output
  double speedChange = P * kp + I * ki + D * kd;  //calculate the correction;


  /*  Serial.print(speedChange);
  Serial.print("\t"); */

  // Apply PID output to control the motors
  motorL->run(FORWARD);
  motorR->run(FORWARD);

  // Adjust motor speeds based on PID output
  leftSpeed = defaultSpeed + speedChange;
  rightSpeed = defaultSpeed - speedChange;

  // Ensure the motor speeds are within valid range (0 to 255)
  leftSpeed = constrain(leftSpeed, 0, FAST_SPEED);
  rightSpeed = constrain(rightSpeed, 0, FAST_SPEED);

  /*
  Serial.print(leftSpeed);
  Serial.print("\t");
  Serial.print(rightSpeed);
  Serial.print("\n");*/

  // Set the motor speeds
  motorL->setSpeed(leftSpeed);
  motorR->setSpeed(rightSpeed);
  delay(10);
}

bool checkStop() {
  if (lastError == error /* && stopped == false*/) {
    return true;
  }
  return false;
}

void task_1() {
  //stopped = true;
  //TO DO
  // Obstacle detected, stop and handle appropriately
  can_counter+=1;
  motorL->run(BACKWARD);
  motorR->run(BACKWARD);
  motorL->setSpeed(0);
  motorR->setSpeed(0);
  delay(500);
  
  if(can_counter%2==0) {
    left_Track_to_Can();
    delay(1000);
    take_trash();
    delay(1000);
    release_trash();
    delay(1000);
    left_Can_to_Track();

  }
  else {
    right_Track_to_Can();
    delay(1000);
    take_trash();
    delay(1000);
    release_trash();
    delay(1000);
    right_Can_to_Track();
  }
  sprint();

  motorL->run(FORWARD);
  motorR->run(FORWARD);
  motorL->setSpeed(SLOW_SPEED);
  motorR->setSpeed(SLOW_SPEED);
  delay(200);
}

void task_2() {
  //stopped = true;
  //TO DO
  // Obstacle detected, stop and handle appropriately
  motorL->run(BACKWARD);
  motorR->run(BACKWARD);
  motorL->setSpeed(0);
  motorR->setSpeed(0);
  delay(2000);

  motorL->run(FORWARD);
  motorR->run(FORWARD);
  motorL->setSpeed(SLOW_SPEED);
  motorR->setSpeed(SLOW_SPEED);
  delay(250);
}

void driveUntilSignalChange() {

  while (lastError == error) {

    lastError = error;
    error = getError();

    // Set the motor speeds
    motorL->setSpeed(SLOW_SPEED);  // drive slowly straight until signal changes
    motorR->setSpeed(SLOW_SPEED);  // drive slowly straight until signal changes
  }
  return;
}

////////////////////////////////////////////////////////////////////////////////////////
//Main code: functions setup() and loop()

void setup() {
  pinMode(R_S, INPUT);
  pinMode(CR_S, INPUT);
  pinMode(CL_S, INPUT);
  pinMode(L_S, INPUT);

  Serial.begin(9600);

  AFMS.begin();
  motorR->setSpeed(defaultSpeed);
  motorL->setSpeed(defaultSpeed);

  myservo1.attach(10);
  myservo2.attach(9);

  // Start position servo
  for (pos = 30; pos >= 90; pos += 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  delay(1000);
}

void loop() {

  lastError = error;
  error = getError();

  if (error == 1 && checkStop()) {  // Task 1, but check 2 times
    task_1();
    //   stopped = true;
    driveUntilSignalChange();
  } else if (error == 2 && checkStop()) {  // Task 2, but check 2 times
    task_2();
    //    stopped = true;
    driveUntilSignalChange();
  } else if (error == 3 && lastError == error) {  // Stop, but check 2 times
    motorR->setSpeed(0);
    motorL->setSpeed(0);
  } else {  // Proceed with line tracking
    PID_line_tracking();
  }
}