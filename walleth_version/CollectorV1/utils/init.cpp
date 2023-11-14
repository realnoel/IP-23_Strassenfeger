#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

#include "init.h"

void take_trash(int min_pos, int max_pos, Servo myservo) {
    for (pos = min_pos; pos <= max_pos; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
}
void release_trash(int min_pos, int max_pos, Servo myservo) {
    for (pos = max_pos; pos >= min_pos; pos -= 1) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
}

// Movement to can/track
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

// Other movement
void sprint(Adafruit_DCMotor motorR, Adafruit_DCMotor motorL) {
    motorR->run(FORWARD);
    motorL->run(FORWARD);
    delay(800);
    motorR->run(RELEASE);
    motorL->run(RELEASE);
}