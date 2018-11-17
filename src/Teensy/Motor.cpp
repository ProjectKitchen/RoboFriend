#include <Arduino.h>
#include <TimerThree.h>
#include "GPIO.h"
#include "Motor.h"
#include "Sensor.h"
#include "support.h"

Motor::Motor() {}
Motor::~Motor() {}

extern long loopcounter;

void Motor::init() {
    // TODO move to a separate init function
    pinMode(PIN_OC, OUTPUT);
    digitalWrite(PIN_OC, HIGH);
  
    Timer3.initialize(100);   // 100us period = 10kHz frequency for PWM pins  
    Timer3.pwm(PIN_MT_RI_PWM, 0);
    Timer3.pwm(PIN_MT_LE_PWM, 0);

    pinMode(PIN_MT_RI_BCK, OUTPUT);
    pinMode(PIN_MT_RI_FWD, OUTPUT);
    pinMode(PIN_MT_RI_PWM, OUTPUT);

    pinMode(PIN_MT_LE_FWD, OUTPUT);
    pinMode(PIN_MT_LE_BCK, OUTPUT);
    pinMode(PIN_MT_LE_PWM, OUTPUT);

#if DEBUG
    /* FORWARD */
    digitalWrite(PIN_MT_LE_FWD, HIGH);
    digitalWrite(PIN_MT_RI_FWD, HIGH);
    digitalWrite(PIN_MT_LE_BCK, LOW);
    digitalWrite(PIN_MT_RI_BCK, LOW);
    Timer3.pwm(PIN_MT_RI_PWM, 1023);
    Timer3.pwm(PIN_MT_LE_PWM, 1023);
    delay(5000);

    /* RIGHT */
    digitalWrite(PIN_MT_LE_FWD, LOW);
    digitalWrite(PIN_MT_RI_FWD, HIGH);
    digitalWrite(PIN_MT_LE_BCK, LOW);
    digitalWrite(PIN_MT_RI_BCK, LOW);
    delay(5000);

    /* LEFT */
    digitalWrite(PIN_MT_LE_FWD, HIGH);
    digitalWrite(PIN_MT_RI_FWD, LOW);
    digitalWrite(PIN_MT_LE_BCK, LOW);
    digitalWrite(PIN_MT_RI_BCK, LOW);
    delay(5000);

    /* BACKWARD */
    digitalWrite(PIN_MT_LE_FWD, LOW);
    digitalWrite(PIN_MT_RI_FWD, LOW);
    digitalWrite(PIN_MT_LE_BCK, HIGH);
    digitalWrite(PIN_MT_RI_BCK, HIGH);
    delay(5000);

    digitalWrite(PIN_MT_LE_FWD, LOW);
    digitalWrite(PIN_MT_RI_FWD, LOW);
    digitalWrite(PIN_MT_LE_BCK, LOW);
    digitalWrite(PIN_MT_RI_BCK, LOW);
    Timer3.pwm(PIN_MT_RI_PWM, 0);
    Timer3.pwm(PIN_MT_LE_PWM, 0);
#endif

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
  
  if (leftSpeed == 0) { 
    digitalWrite(PIN_MT_LE_FWD, LOW); 
    digitalWrite(PIN_MT_LE_BCK, LOW); 
  }
  else if (leftSpeed > MOVE_THRESHOLD)  { digitalWrite(PIN_MT_LE_BCK, LOW); digitalWrite(PIN_MT_LE_FWD, HIGH); }
  else if (leftSpeed < -MOVE_THRESHOLD) { digitalWrite(PIN_MT_LE_FWD, LOW); digitalWrite(PIN_MT_LE_BCK, HIGH); }
  
  if (rightSpeed == 0) { digitalWrite(PIN_MT_RI_FWD, LOW); digitalWrite(PIN_MT_RI_FWD, LOW); }
  else if (rightSpeed > MOVE_THRESHOLD)  { digitalWrite(PIN_MT_RI_BCK, LOW); digitalWrite(PIN_MT_RI_FWD, HIGH); }
  else if (rightSpeed < -MOVE_THRESHOLD) { digitalWrite(PIN_MT_RI_FWD, LOW); digitalWrite(PIN_MT_RI_BCK, HIGH); }

  if (rightSpeed) {
    Timer3.pwm(PIN_MT_RI_PWM, map((rightSpeed<0) ? -rightSpeed : rightSpeed,0,255,20,600));   // right motor slighty slower: compensate PWMs!
  }
  else Timer3.pwm(PIN_MT_RI_PWM,0);
  
  if (leftSpeed)
     Timer3.pwm(PIN_MT_LE_PWM, map((leftSpeed<0) ? -leftSpeed : leftSpeed,0,255,20,512));
  else Timer3.pwm(PIN_MT_LE_PWM,0);

  if (PRINT_MOTORSPEED_MESSAGES) {
    if (((loopcounter % 3) == 0) && (leftSpeed || rightSpeed))
      Serial.printf("Speed: %d/%d\n",rightSpeed,leftSpeed);
  }
}


void Motor::stop()
{
    analogWrite(PIN_MT_LE_PWM, 0);
    analogWrite(PIN_MT_RI_PWM, 0);

    digitalWrite(PIN_MT_LE_FWD, LOW);
    digitalWrite(PIN_MT_RI_BCK, LOW);
    digitalWrite(PIN_MT_LE_BCK, LOW);
    digitalWrite(PIN_MT_RI_FWD, LOW);

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
      if (PRINT_SENSOR_MESSAGES) {
         if ((loopcounter % 10) == 0) Serial.printf("Right %04d ",Sensors.getIRSensorRightValue());
      }
    }
    if (Sensors.isIRSensorLeftTriggered()) {
      triggered=1;
      if (leftSpeed>0) leftSpeed=0;
      if (PRINT_SENSOR_MESSAGES) {
         if ((loopcounter % 10) == 0) Serial.printf("Left %04d ",Sensors.getIRSensorLeftValue());
      }
    }
    if (Sensors.isIRSensorMiddleTriggered()) {
      triggered = 1;
      if (rightSpeed > 0) rightSpeed = 0;
      if (leftSpeed > 0) leftSpeed = 0;
      if (PRINT_SENSOR_MESSAGES) {
         if ((loopcounter % 10) == 0) Serial.printf("Middle %04d ",Sensors.getIRSensorMiddleValue());
      }
    }
    if (PRINT_SENSOR_MESSAGES) {
      if (((loopcounter % 10) == 0) && triggered) Serial.println("");
    }

    return(triggered);
}
