#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Set pin
#define R_S A0  // Infrared sensor Right
#define CR_S A1 // Infrared sensor Center Right
#define CL_S A2 // Infrared sensor Center Left
#define L_S A3  // Infrared sensor Left

// Set speed
#define SLOW_SPEED 150
// #define MEDIUM_SPEED 55
#define FAST_SPEED 244
// Benutze TRASH_SPEED wenn der Roboter auf Tonnen zufährt
#define TRASH_SPEED 75

// Hier ist die Strecke modelliert
// Track 0 left - straight, 1 right - straight, numbers unequal to 0 or 1 belong to positions on the track
int competition_track[] = {0, 1, 0, 4, 0, 1, 0, 8, 9, 2, 2, 3, 2, 0, 1, 0, 3, 2, 3};

// Track 0 left - straight, 1 right - straight
//int competition_track_simple[] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1};
//int competition_track[] = {0,4};

int defaultSpeed = 70;
int stoptime = 0;

int can_counter = 0;

// Hier stehen die Positionen des Arms in Grad
int pos_unten = 168;
int pos_oben = 70;

////////////////////////////////////////////////////////////////////////////////////////
// PID Controller

// Gain k=[0;1]
// Tuning parameter:
// Start with kp=1 and decrease value to stable system
// Then increase ki and kd if needed
// double kp = 0.22;  // Proportional gain
// double ki = 0.022; // Integral gain
// double kd = 0;    // Derivative gain *************************** possible delete
double kp = 0;    // Proportional gain
double ki = 0.005;  // Integral gain
double kd = 0;    // Derivative gain *************************** possible delete



// Initialize PID components to 0
double P = 0; // Proportional component of the control output
double I = 0; // Integral component of the control output
double D = 0; // Derivative component of the control output

int error;
int previousError = 0;
int smallError = defaultSpeed * 0.50; // Need to calibrate this parameter
int mediumError = defaultSpeed * 1.0;  // Need to calibrate this parameter
int bigError = defaultSpeed * 2.2;   // Need to calibrate this parameter
bool stopped = false;
bool skip_line = false;

int lastError = 0;
int lastLineSignal = 0;

int leftSpeed = 0;
int rightSpeed = 0;

////////////////////////////////////////////////////////////////////////////////////////

// Hier werden die Motoren initalisiert
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorR = AFMS.getMotor(2);
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorArm = AFMS.getMotor(3);

Servo servoArm;

// Diese Funktion bewegt den Arm nach oben
// PRE: servo -> anzusteuernder Servo
// PRE: unten Startposition des Arms in Grad, oben Endposition des Arms in Grad
void arm_up(Servo servo, int unten, int oben)
{
    for (int pos = unten; pos >= oben; pos -= 1)
    { 
        servo.write(pos); // tell servo to go to position in variable 'pos'
        delay(10);        // waits 15ms for the servo to reach the position
        Serial.write(servoArm.read());
        Serial.write("\n");
    }
}

// Diese Funktion bewegt den Arm nach oben
// PRE: servo -> anzusteuernder Servo
// PRE: unten Endposition des Arms in Grad, oben Startposition des Arms in Grad
void arm_down(Servo servo, int unten, int oben)
{
    for (int pos = oben; pos <= unten; pos += 1)
    {
        if (pos < unten - 20)
        {
          servo.write(pos);
          delay(10);
        }
        else
        {
          servo.write(pos);
          delay(20);
        }
        
        Serial.write(servoArm.read());
        Serial.write("\n");
    }
}

void arm_down_and_drive(Servo servo, int unten, int oben)
{
  for (int pos = oben; pos <= unten; pos += 1)
    {
        servo.write(pos);
        delay(5);
        error = getError();
        PID_line_tracking();
    }
}

// Diese Funktion schliesst den Greifer
// PRE: anzusteuernder Motor
void trash_grab(Adafruit_DCMotor *motor)
{
    motor->run(FORWARD);
    delay(1000);
}

// Diese Funktion öffnet den Greifer
// PRE: anzusteuernder Motor
void trash_release(Adafruit_DCMotor *motor)
{
    motor->run(BACKWARD);
    delay(1000);
    motor->run(RELEASE);
}

// Diese Funktion führt einen kompletten Greifprozess aus 
// Zugreifen -> Arm hoch -> Arm runter -> Arm öffnen
// PRE: motor: anzusteuernder Motor, servo: anzusteuernder Servo
// PRE: unten: Startposition des Arms in Grad, oben: Endposition des Arms in Grad
void full_arm_movement(Adafruit_DCMotor *motor, Servo servo, int unten, int oben)
{
    trash_grab(motor);
    delay(100);
    arm_up(servo, unten, oben);
    delay(1000);
    // Becher leeren funktion hinzufügen
    arm_down(servo, unten, oben);
    delay(100);
    trash_release(motor);
}

// Diese Position setzt den Arm auf seine Fahrtposition
// PRE: motor: anzusteuernder Motor, servo: anzusteuernder Servo
void initalize_arm(Servo servo, Adafruit_DCMotor *motor)
{
    arm_down(servo, pos_unten, pos_unten);
    trash_release(motor);
}

////////////////////////////////////////////////////////////////////////////
// Diese Funktionen funktionieren nur für Tonnen auf gerader Strecke

void left_Track_to_Can(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
    motorR->setSpeed(TRASH_SPEED);
    motorL->setSpeed(TRASH_SPEED);

    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(1300);
    motorR->run(FORWARD);
    motorL->run(BACKWARD);
    delay(250);
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(550);
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
    delay(330);
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
    delay(315);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
}

void right_Track_to_Can(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
    motorR->setSpeed(TRASH_SPEED);
    motorL->setSpeed(TRASH_SPEED);

    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(1300);
    motorR->run(BACKWARD);
    motorL->run(FORWARD);
    delay(275);
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(550);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
}


////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// Becher auf Position 8

void pos4_Can_to_Track(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
    arm_up(servoArm, pos_unten, pos_oben);
    motorR->run(FORWARD);
    motorL->run(BACKWARD);
    delay(500);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
}

void pos4_Track_to_Can(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
    motorR->setSpeed(TRASH_SPEED);
    motorL->setSpeed(TRASH_SPEED);

    motorR->run(BACKWARD);
    motorL->run(FORWARD);
    delay(125);
    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(1200);
    motorR->run(BACKWARD);
    motorL->run(FORWARD);
    delay(225);
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(380);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
}

////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// Becher auf Position 4

void pos8_Can_to_Track(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(300);
    motorR->run(FORWARD);
    motorL->run(BACKWARD);
    delay(315);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
}

void pos8_Track_to_Can(Adafruit_DCMotor *motorR, Adafruit_DCMotor *motorL)
{
    motorR->setSpeed(TRASH_SPEED);
    motorL->setSpeed(TRASH_SPEED);

    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(1300);
    motorR->run(BACKWARD);
    motorL->run(FORWARD);
    delay(250);
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(650);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
}
////////////////////////////////////////////////////////////////////////////


void ignore_stop_line(int duration)
{
    for (int i = 0; i < duration / 10; i++) // One iteration is 10ms
    {
        error = getError_ignore_stop_line();
        PID_line_tracking();
    }
}
int getError_ignore_stop_line()
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
    return 0; // Default case: Unknown/error state. Proceed with line tracking
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
    {             // Special case: Only the center sensor detects a bright surface (possible obstacle)
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

void task_1()
{
    stopped = true;

    motorL->run(BACKWARD);
    motorR->run(BACKWARD);
    motorL->setSpeed(0);
    motorR->setSpeed(0);

    if (competition_track[can_counter] == 0)
    {
        left_Track_to_Can(motorR, motorL);
        full_arm_movement(motorArm, servoArm, pos_unten, pos_oben);
        left_Can_to_Track(motorR, motorL);
        ignore_stop_line(1000);
    }
    else if (competition_track[can_counter] == 4)
    {
      pos4_Track_to_Can(motorR, motorL);
      full_arm_movement(motorArm, servoArm, pos_unten, pos_oben);
      pos4_Can_to_Track(motorR, motorL);
      ignore_stop_line(1500);
      arm_down_and_drive(servoArm, pos_unten, servoArm.read());
    } 
    else
    {
      right_Track_to_Can(motorR, motorL);
      full_arm_movement(motorArm, servoArm, pos_unten, pos_oben);
      right_Can_to_Track(motorR, motorL);
      ignore_stop_line(1000);
    }

    
    can_counter++;
}

void task_2()
{
    // stopped = true;
    // TO DO
    //  Obstacle detected, stop and handle appropriately
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

    // Serial.begin(9600);

    AFMS.begin();
    motorR->setSpeed(defaultSpeed);
    motorL->setSpeed(defaultSpeed);
    motorArm->setSpeed(244);

    servoArm.attach(10);
    initalize_arm(servoArm, motorArm); // ARM DOWN, GRIPPER OPEN
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