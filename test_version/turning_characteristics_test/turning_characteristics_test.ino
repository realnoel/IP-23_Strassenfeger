#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////////////////
// Set pin
#define R_S A0  // Infrared sensor Right
#define CR_S A1 // Infrared sensor Center Right
#define CL_S A2 // Infrared sensor Center Left
#define L_S A3  // Infrared sensor Left

// Set speed
#define SLOW_SPEED 45
// #define MEDIUM_SPEED 55
#define FAST_SPEED 55

// Track 0 left - straight, 1 right - straight, 2 left - curve, 3 right - curve
int competition_track[] = {0, 1, 0, 1, 3, 0, 1, 0, 3, 2, 2, 3, 2, 0, 1, 0, 3, 2, 3};

// Track 0 left - straight, 1 right - straight
int competition_track_simple[] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1};


int defaultSpeed = 50;
int stoptime = 0;

int can_counter = 0;

// Servo code
int min_pos = 0;
int max_pos = 110;
int pos = 0;


  ////////////////////////////////////////////////////////////////////////////////////////
  // PID Controller

  // Gain k=[0;1]
  // Tuning parameter:
  // Start with kp=1 and decrease value to stable system
  // Then increase ki and kd if needed
  double kp = 0.29;  // Proportional gain
  double ki = 0.027; // Integral gain
  double kd = 0;     // Derivative gain *************************** possible delete

  // Initialize PID components to 0
  double P = 0; // Proportional component of the control output
  double I = 0; // Integral component of the control output
  double D = 0; // Derivative component of the control output

  int error;
  int previousError = 0;
  int smallError = defaultSpeed * 0.5; // Need to calibrate this parameter
  int mediumError = defaultSpeed * 1;  // Need to calibrate this parameter
  int bigError = defaultSpeed * 2.2;   // Need to calibrate this parameter
  bool stopped = false;

  int lastError = 0;
  int lastLineSignal = 0;

  int leftSpeed = 0;
  int rightSpeed = 0;


////////////////////////////////////////////////////////////////////////////////////////
// Functions

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorArm = AFMS.getMotor(3);

Servo servoArm;

void arm_down(Servo servo, int min_pos, int max_pos)
{
  for (int pos = min_pos; pos <= max_pos; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos); // tell servo to go to position in variable 'pos'
    delay(25);        // waits 15ms for the servo to reach the position
  }
}
void arm_up(Servo servo, int min_pos, int max_pos)
{
  for (int pos = max_pos; pos >= min_pos; pos -= 1)
  {
    servo.write(pos);
    delay(25);
  }
}

void trash_grap(Adafruit_DCMotor *motor)
{
  motor->run(BACKWARD);
  motor->run(BACKWARD);
  motor->setSpeed(100);
  motor->setSpeed(100);
  delay(750);
}
void trash_release(Adafruit_DCMotor *motor)
{
  motor->run(FORWARD);
  motor->run(FORWARD);
  motor->setSpeed(100);
  motor->setSpeed(100);
  delay(750);
}
void full_arm_movement(Adafruit_DCMotor *motor, Servo servo, int min_pos, int max_pos)
{
  trash_grap(motor);
  delay(500);
  arm_up(servo, min_pos, max_pos);
  delay(1000);
  arm_down(servo, min_pos, max_pos);
  delay(1000);
  trash_release(motor);
}

void initalize_arm(Servo servo, Adafruit_DCMotor *motor)
{

  arm_down(servo, min_pos, servo.read());
  trash_release(motor);
}

void right_Track_to_Can(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
  motorR->run(BACKWARD);
  motorL->run(BACKWARD);
  motorR->setSpeed(FAST_SPEED);
  motorL->setSpeed(FAST_SPEED);
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
void right_Can_to_Track(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
  motorR->run(BACKWARD);
  motorL->run(BACKWARD);
  delay(300);
  motorR->run(FORWARD);
  motorL->run(BACKWARD);
  delay(400);
  motorR->run(RELEASE);
  motorL->run(RELEASE);
}

void left_Track_to_Can(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
  motorR->run(BACKWARD);
  motorL->run(BACKWARD);
  motorR->setSpeed(FAST_SPEED);
  motorL->setSpeed(FAST_SPEED);
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

void left_Can_to_Track(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
  motorR->run(BACKWARD);
  motorL->run(BACKWARD);
  delay(300);
  motorR->run(BACKWARD);
  motorL->run(FORWARD);
  delay(400);
  motorR->run(RELEASE);
  motorL->run(RELEASE);
}

void sprint(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
  motorR->run(FORWARD);
  motorL->run(FORWARD);
  delay(800);
  motorR->run(RELEASE);
  motorL->run(RELEASE);
}

// Function to get the error from the line-following sensors (0-2).
int getError()
{
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
  if (sensorL == 0 && sensorCL == 1 && sensorCR == 1 && sensorR == 1)
  {
    Serial.println("bigError turnLeft");
    return -bigError; // The robot is significantly to the right of the line
  }
  else if (sensorL == 0 && sensorCL == 0 && sensorCR == 1 && sensorR == 1)
  {
    Serial.println("mediumError turnLeft");
    return -mediumError; // The robot is the right of the line
  }
  else if (sensorL == 0 && sensorCL == 0 && sensorCR == 0 && sensorR == 1)
  {
    Serial.println("smallError turnLeft");
    return -smallError; // The robot is slightly to the right of the line
  }
  else if (sensorL == 1 && sensorCL == 0 && sensorCR == 0 && sensorR == 1)
  {
    Serial.println("noError");
    I = 0;
    return 0; // The robot is on the line (centered)
  }
  else if (sensorL == 1 && sensorCL == 0 && sensorCR == 0 && sensorR == 0)
  {
    Serial.println("smallError turnRight");
    return smallError; // The robot is slightly to the left of the line
  }
  else if (sensorL == 1 && sensorCL == 1 && sensorCR == 0 && sensorR == 0)
  {
    Serial.println("mediumError turnRight");
    return mediumError; // The robot is the left of the line
  }
  else if (sensorL == 1 && sensorCL == 1 && sensorCR == 1 && sensorR == 0)
  {
    Serial.println("bigError turnRight");
    return bigError; // The robot is significantly to the left of the line
  }
  else if (sensorL == 0 && sensorCL == 0 && sensorCR == 0 && sensorR == 0)
  {
    return 1; // Return obstacle 1
  }
  else if ((sensorL == 0 && sensorCL == 1 && sensorCR == 1 && sensorR == 0) || (sensorL == 0 && sensorCL == 1 && sensorCR == 0 && sensorR == 0) || (sensorL == 0 && sensorCL == 0 && sensorCR == 1 && sensorR == 0))
  {           // Special case: Only the center sensor detects a bright surface (possible obstacle)
    return 2; // Return obstacle 2
  }
  else if (sensorL == 1 && sensorCL == 1 && sensorCR == 1 && sensorR == 1)
  { // Special case: All sensors detect a white surface (end of track)
    return 3;
  }
  return 0; // Default case: Unknown/error state. Proceed with line tracking
}

void PID_line_tracking()
{
  // stopped = false;

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
  double speedChange = P * kp + I * ki + D * kd; // calculate the correction;

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

bool checkStop()
{
  if (lastError == error /* && stopped == false*/)
  {
    return true;
  }
  return false;
}



void driveUntilSignalChange()
{

  while (lastError == error)
  {

    lastError = error;
    error = getError();

    // Set the motor speeds
    motorL->setSpeed(SLOW_SPEED); // drive slowly straight until signal changes
    motorR->setSpeed(SLOW_SPEED); // drive slowly straight until signal changes
  }
  return;
}

////////////////////////////////////////////////////////////////////////////////////////
// Main code: functions setup() and loop()

void setup()
{
  pinMode(R_S, INPUT);
  pinMode(CR_S, INPUT);
  pinMode(CL_S, INPUT);
  pinMode(L_S, INPUT);

  Serial.begin(9600);

  AFMS.begin();
  motorR->setSpeed(defaultSpeed);
  motorL->setSpeed(defaultSpeed);
  motorArm->setSpeed(100);

  servoArm.attach(9);

  initalize_arm(servoArm, motorArm);
}

void loop()
{
  motorR->run(RELEASE);
  motorL->run(RELEASE);
  motorR->setSpeed(FAST_SPEED);
  motorL->setSpeed(FAST_SPEED);
  delay(100);

  motorR->setSpeed(75);
  motorL->setSpeed(75);

  motorR->run(FORWARD);
  motorL->run(BACKWARD);

  delay(200);
  
  motorR->setSpeed(0);
  motorL->setSpeed(0); 

  delay(10000);



  // for(int i = 50;i<=500;i+50) {

  //   motorR->setSpeed(50);
  //   motorL->setSpeed(50);

  //   motorR->run(FORWARD);
  //   motorL->run(BACKWARD);

  //   delay(i);

  //   motorR->setSpeed(0);
  //   motorL->setSpeed(0); 

  //   delay(10000);
  // }
  
}