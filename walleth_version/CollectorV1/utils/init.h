#ifndef INIT_H
#define INIT_H

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Movement with trash
void take_trash(int min_pos, int max_pos, Servo myservo);
void release_trash(int min_pos, int max_pos, Servo myservo);

// Movement to Can/Track
void right_Track_to_Can(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL);
void right_Can_to_Track(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL);
void left_Track_to_Can(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL);
void left_Can_to_Track(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL);

// Other movement methods
void sprint(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL)


#endif