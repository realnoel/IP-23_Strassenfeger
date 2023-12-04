#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
  
  
  void right_Track_to_Can(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL) {
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
  void right_Can_to_Track(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL) {
    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(300);
    motorR->run(FORWARD);
    motorL->run(BACKWARD);
    delay(400);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
  }

  void left_Track_to_Can(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL) {
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

  void left_Can_to_Track(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL) {
    motorR->run(BACKWARD);
    motorL->run(BACKWARD);
    delay(300);
    motorR->run(BACKWARD);
    motorL->run(FORWARD);
    delay(400);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
  }

  void sprint(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL) {
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(800);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
  }