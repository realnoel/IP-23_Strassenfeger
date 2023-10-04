#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // 1,2 ... -> M1,M2 ... on board
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);

void circle() { // 2 is left, 3 is right
  for(int i = 0;i<4;i++) {
    //forwards
    myMotor2->setSpeed(200);
    myMotor2->run(FORWARD);
    myMotor3->setSpeed(200);
    myMotor3->run(FORWARD);
    delay(2000);
    //right
    myMotor2->setSpeed(100);
    myMotor3->setSpeed(50);
    delay(1000);
  }
}

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();

  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);

  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Enter an integer to start ... ");

  while (Serial.available() == 0) {
  }

  int input = Serial.parseInt();

  if(input != 0) {
    circle();
  }

  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
}
