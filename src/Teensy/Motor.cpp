#include <Arduino.h>
#include <TimerThree.h>
#include "Motor.h"
#include "Sensor.h"

Motor::Motor() {}
Motor::~Motor() {}

void Motor::init() {
    Timer3.initialize(40);   // 40us period = 25kHz analogWrite frequency for PWM pins  

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

  if (intendedLeftSpeed > leftSpeed+ACCEL_STEP) leftSpeed += ACCEL_STEP;
  else if (intendedLeftSpeed < leftSpeed-ACCEL_STEP) leftSpeed -= ACCEL_STEP;
  else leftSpeedReached=true;

  if (intendedRightSpeed > rightSpeed+ACCEL_STEP) rightSpeed += ACCEL_STEP;
  else if (intendedRightSpeed < rightSpeed-ACCEL_STEP) rightSpeed -= ACCEL_STEP;
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

  analogWrite(RightPWMPin, (rightSpeed<0) ? -rightSpeed : rightSpeed);
  analogWrite(LeftPWMPin, (leftSpeed<0) ? -leftSpeed : rightSpeed);
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

void Motor::drive(int right, int left, int duration) {
   intendedRightSpeed=right;
   intendedLeftSpeed=left;
   intendedDuration=duration;
}

void Motor::handleObstacles() {
    if (Sensors.isIRSensorRightTriggered()) {
      if (rightSpeed>0) rightSpeed=0;
    }
    if (Sensors.isIRSensorLeftTriggered()) {
      if (leftSpeed>0) leftSpeed=0;
    }
    if (Sensors.isIRSensorMiddleTriggered()) {
      if (rightSpeed>0) rightSpeed=0;
      if (leftSpeed>0) leftSpeed=0;
    }
}
