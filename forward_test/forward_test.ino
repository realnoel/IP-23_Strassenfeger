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

void loop() {
  // put your main code here, to run repeatedly:
  motorR->run(FORWARD);
  motorL->run(FORWARD);
  motorR->setSpeed(50);
  motorL->setSpeed(50);
}
