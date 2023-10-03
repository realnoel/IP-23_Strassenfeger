#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(2); // 1,2 ... -> M1,M2 ... on board

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  AFMS.begin();
  myMotor->setSpeed(0);
  myMotor->run(FORWARD);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("Hello World!");
  for (int i = 0;i<=255;i++) {
    myMotor->setSpeed(i) // 0 (stopped) to 255 (full speed)
  }
  myMotor->setSpeed(0);
  delay(1000); // in ms
  myMotor->setSpeed(100);
  delay(1000);
}
