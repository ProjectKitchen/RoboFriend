#include <Arduino.h>
#include <TimerThree.h>
#include "GPIO.h"
#include "Motor.h"
#include "Sensor.h"
#include "support.h"


#define HANDLE_OBSTACLES 0       // currently disables because sensor values are not valid in Robofriend V2 !
#define ROBOFRIEND_VERSION2      // version 1 and version 2 use different motor compensation settings

#ifdef ROBOFRIEND_VERSION2
  int LEFT_MOTOR_MINPWM =20;
  int RIGHT_MOTOR_MINPWM = 20;
  int RIGHT_MOTOR_MAXPWM = 1500; // actually a big difference for the right motor! - a motor speed control loop is mandatory !
  int LEFT_MOTOR_MAXPWM = 512;
#else
  int LEFT_MOTOR_MINPWM = 20;
  int RIGHT_MOTOR_MINPWM = 20;
  int RIGHT_MOTOR_MAXPWM = 600;
  int LEFT_MOTOR_MAXPWM = 512;
#endif

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

  if (HANDLE_OBSTACLES) handleObstacles();
  
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
    Timer3.pwm(PIN_MT_RI_PWM, map((rightSpeed<0) ? -rightSpeed : rightSpeed,0,255,LEFT_MOTOR_MINPWM,RIGHT_MOTOR_MAXPWM));   // right motor slighty slower: compensate PWMs!
  }
  else Timer3.pwm(PIN_MT_RI_PWM,0);
  
  if (leftSpeed)
     Timer3.pwm(PIN_MT_LE_PWM, map((leftSpeed<0) ? -leftSpeed : leftSpeed,0,255,LEFT_MOTOR_MINPWM,LEFT_MOTOR_MAXPWM));
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
   Serial.printf("Drive right=%04d, left=%04d, duration=%04d\n",left,right,duration);
   intendedRightSpeed=right;
   intendedLeftSpeed=left;
   intendedDuration=duration;
}

int Motor::handleObstacles() {
    int triggered=0;

    if (sensors.isIRSensorRightTriggered()) {
      triggered=1;
      if (rightSpeed>0) rightSpeed=0;
      if (PRINT_SENSOR_MESSAGES) {
         if ((loopcounter % 10) == 0) Serial.printf("Right %04d ",sensors.getIRSensorRightValue());
      }
    }
    if (sensors.isIRSensorLeftTriggered()) {
      triggered=1;
      if (leftSpeed>0) leftSpeed=0;
      if (PRINT_SENSOR_MESSAGES) {
         if ((loopcounter % 10) == 0) Serial.printf("Left %04d ",sensors.getIRSensorLeftValue());
      }
    }
    if (sensors.isIRSensorMiddleTriggered()) {
      triggered = 1;
      if (rightSpeed > 0) rightSpeed = 0;
      if (leftSpeed > 0) leftSpeed = 0;
      if (PRINT_SENSOR_MESSAGES) {
         if ((loopcounter % 10) == 0) Serial.printf("Middle %04d ",sensors.getIRSensorMiddleValue());
      }
    }
    if (PRINT_SENSOR_MESSAGES) {
      if (((loopcounter % 10) == 0) && triggered) Serial.println("");
    }

    return(triggered);
}
