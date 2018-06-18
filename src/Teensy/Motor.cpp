#include <Arduino.h>
#include <TimerThree.h>
#include "Motor.h"
#include "Sensor.h"

Motor::Motor() {}
Motor::~Motor() {}

extern long loopcounter;

void Motor::init() {
    Timer3.initialize(100);   // 100us period = 10kHz frequency for PWM pins  
    Timer3.pwm(RightPWMPin, 0);
    Timer3.pwm(LeftPWMPin, 0);

    pinMode(RightBackwardPin, OUTPUT);
    pinMode(RightForwardPin, OUTPUT);
    pinMode(RightPWMPin, OUTPUT);

    pinMode(LeftForwardPin, OUTPUT);
    pinMode(LeftBackwardPin, OUTPUT);
    pinMode(LeftPWMPin, OUTPUT);

    intendedLeftSpeed=leftSpeed=0;
    intendedRightSpeed=rightSpeed=0;
    intendedDuration=0;
}


void Motor::updateMotors()
{
  bool leftSpeedReached=false,rightSpeedReached=false;
    
  if (intendedLeftSpeed >= leftSpeed+ACCEL_STEP) leftSpeed += ACCEL_STEP;
  else if (intendedLeftSpeed <= leftSpeed-ACCEL_STEP) leftSpeed -= ACCEL_STEP;
  else leftSpeedReached=true;

  if (intendedRightSpeed >= rightSpeed+ACCEL_STEP) rightSpeed += ACCEL_STEP;
  else if (intendedRightSpeed <= rightSpeed-ACCEL_STEP) rightSpeed -= ACCEL_STEP;
  else rightSpeedReached=true;

  if (leftSpeedReached && rightSpeedReached && (intendedDuration>0)) {
      intendedDuration--;
      if (!intendedDuration) {
        intendedLeftSpeed=0;
        intendedRightSpeed=0;
    }
  }

  handleObstacles();
  
  if (leftSpeed == 0) { digitalWrite(LeftForwardPin, LOW); digitalWrite(LeftBackwardPin, LOW); }
  else if (leftSpeed > MOVE_THRESHOLD)  { digitalWrite(LeftBackwardPin, LOW); digitalWrite(LeftForwardPin, HIGH); }
  else if (leftSpeed < -MOVE_THRESHOLD) { digitalWrite(LeftForwardPin, LOW); digitalWrite(LeftBackwardPin, HIGH); }
  
  if (rightSpeed == 0) { digitalWrite(RightForwardPin, LOW); digitalWrite(RightForwardPin, LOW); }
  else if (rightSpeed > MOVE_THRESHOLD)  { digitalWrite(RightBackwardPin, LOW); digitalWrite(RightForwardPin, HIGH); }
  else if (rightSpeed < -MOVE_THRESHOLD) { digitalWrite(RightForwardPin, LOW); digitalWrite(RightBackwardPin, HIGH); }

  if (rightSpeed)
    Timer3.pwm(RightPWMPin, map((rightSpeed<0) ? -rightSpeed : rightSpeed,0,255,20,600));   // right motor slighty slower: compensate PWMs!
  else Timer3.pwm(RightPWMPin,0);
  
  if (leftSpeed)
     Timer3.pwm(LeftPWMPin, map((leftSpeed<0) ? -leftSpeed : leftSpeed,0,255,20,512));
  else Timer3.pwm(LeftPWMPin,0);

  if (((loopcounter % 3) == 0) && (leftSpeed || rightSpeed))
    Serial.printf("Speed: %d/%d\n",rightSpeed,leftSpeed);
}


void Motor::stop()
{
    analogWrite(LeftPWMPin, 0);
    analogWrite(RightPWMPin, 0);

    digitalWrite(LeftForwardPin, LOW);
    digitalWrite(RightBackwardPin, LOW);
    digitalWrite(LeftBackwardPin, LOW);
    digitalWrite(RightForwardPin, LOW);

    intendedLeftSpeed=leftSpeed=0;
    intendedRightSpeed=rightSpeed=0;
    intendedDuration=0;
}

void Motor::drive(int left, int right, int duration) {
   intendedRightSpeed=right;
   intendedLeftSpeed=left;
   intendedDuration=duration;
}

int Motor::handleObstacles() {
    int triggered=0;

    if (Sensors.isIRSensorRightTriggered()) {
      triggered=1;
      if (rightSpeed>0) rightSpeed=0;
      //if ((loopcounter % 10) == 0) Serial.printf("Right %04d ",Sensors.getIRSensorRightValue());
    }
    if (Sensors.isIRSensorLeftTriggered()) {
      triggered=1;
      if (leftSpeed>0) leftSpeed=0;
      //if ((loopcounter % 10) == 0) Serial.printf("Left %04d ",Sensors.getIRSensorLeftValue());
    }
    if (Sensors.isIRSensorMiddleTriggered()) {
      triggered=1;
      if (rightSpeed>0) rightSpeed=0;
      if (leftSpeed>0) leftSpeed=0;
      //if ((loopcounter % 10) == 0) Serial.printf("Middle %04d ",Sensors.getIRSensorMiddleValue());
    }
    //if (((loopcounter % 10) == 0) && triggered) Serial.println("");

    return(triggered);
}
