#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
// attach(9) für Servo 2

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorArm = AFMS.getMotor(3);

Servo servoArm;

int min_pos = 0;
int max_pos = 110;

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
  Serial.write("Grap arm\n");
  trash_grap(motor);
  delay(500);
  Serial.write("Arm up\n");
  arm_up(servo, min_pos, max_pos);
  delay(1000);
  Serial.write("Arm down\n");
  arm_down(servo, min_pos, max_pos);
  delay(1000);
  Serial.write("Release Arm\n");
  trash_release(motor);
}

void initalize_arm(Servo servo, Adafruit_DCMotor *motor) {
  //angleCurrent.println(servo.read());
  arm_down(servo, min_pos, servo.read());
  //angleCurrent.println(servo.read());
  Serial.write("Arm in setup position\n");
  trash_release(motor);
  Serial.write("Arm open in setup position\n");
}

void setup()
{
  //beginCommand.begin(4800);
  //angleCurrent.begin(4800);

  AFMS.begin();
  servoArm.attach(9);

  Serial.write("Initalized\n");
  initalize_arm(servoArm, motorArm);
}

void loop()
{
  full_arm_movement(motorArm, servoArm, min_pos, max_pos);
}
