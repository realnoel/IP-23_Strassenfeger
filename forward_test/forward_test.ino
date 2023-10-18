#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorR = AFMS.getMotor(2); 
Adafruit_DCMotor *motorL = AFMS.getMotor(1);

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();

  motorR->run(RELEASE);
  motorL->run(RELEASE);

}

void right_Track_to_Can() {
  motorR->run(BACKWARD);
  motorL->run(BACKWARD);
  motorR->setSpeed(50);
  motorL->setSpeed(50);
  delay(750);
  motorR->run(BACKWARD);
  motorL->run(FORWARD);
  delay(750);
  motorR->run(FORWARD);
  motorL->run(FORWARD);
  delay(500);
  motorR->run(RELEASE);
  motorL->run(RELEASE);
}
void right_Can_to_Track() {
  motorR->run(BACKWARD);
  motorL->run(BACKWARD);
  delay(750);
  motorR->run(FORWARD);
  motorL->run(BACKWARD);
  delay(700);
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
  delay(750);
  motorR->run(FORWARD);
  motorL->run(FORWARD);
  delay(500);
  motorR->run(RELEASE);
  motorL->run(RELEASE);
}

void left_Can_to_Track() {
  motorR->run(BACKWARD);
  motorL->run(BACKWARD);
  delay(750);
  motorR->run(BACKWARD);
  motorL->run(FORWARD);
  delay(750);
  motorR->run(RELEASE);
  motorL->run(RELEASE);
}


void loop() {
  // put your main code here, to run repeatedly:
  right_Track_to_Can();
  delay(2000);
  right_Can_to_Track();

  motorR->run(FORWARD);
  motorL->run(FORWARD);
  delay(3000);

  left_Track_to_Can();
  delay(2000);
  left_Can_to_Track();

  delay(1000000);
}
