#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // 1,2 ... -> M1,M2 ... on board
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);

Adafruit_MS_PWMServoDriver AFSV = Adafruit_MS_PWMServoDriver(1);
Adafruit_MS_PWMServoDriver *myServo1 = AFSV.begin(1);

void setup() {
  // put your setup code here, to run once:

  AFMS.begin();
  AFSV.begin(1);

  myMotor2->setSpeed(0);
  myMotor2->run(FORWARD);

  myMotor3->setSpeed(0);
  myMotor3->run(FORWARD);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("Hello World!");

  myMotor2->setSpeed(255);
  myMotor3->setSpeed(255);

  delay(2000);
  
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
}
