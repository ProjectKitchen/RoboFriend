#include <Arduino.h>
#include <TimerThree.h>
#include "GPIO.h"
#include "Motor.h"
#include "Sensor.h"
#include "support.h"
#include "PID_v1.h"
#include "Odometry.h"
#include "motorMapping.h"


#define ROBOFRIEND_VERSION2      // version 1 and version 2 use different motor compensation settings

#define USE_PID_CONTROL 0        // 1 for PID, 0 for linear
#define HANDLE_OBSTACLES 0       // currently disabled because sensor values are not valid in Robofriend V2 !

#define PRINT_MOTORSPEED_MESSAGES 0  // print every nth interation (0=disable)
#define PRINT_SENSOR_MESSAGES     0

#define MAX_ODOM 180

//tuning parameters for PID motor control  - TBD !
//double KpLeft=1.0,  KiLeft=0.05,  KdLeft=0;
//double KpRight=1.0, KiRight=0.05, KdRight=0;

double KpLeft=0.5,  KiLeft=0.09,  KdLeft=0.02;
double KpRight=0.5, KiRight=0.09, KdRight=0.02;


//variables for PID motor control
double SetpointLeft,  InputLeft,  OutputLeft;
double SetpointRight, InputRight, OutputRight;


// PID controllers
PID leftMotorPID(&InputLeft, &OutputLeft, &SetpointLeft, KpLeft, KiLeft, KdLeft, DIRECT);
PID rightMotorPID(&InputRight, &OutputRight, &SetpointRight, KpRight, KiRight, KdRight, DIRECT);


// for getting the odometry data
extern Odometry odo;
int set_left_odom;
int set_right_odom;

 bool leftSpeedReached=false,rightSpeedReached=false;

#ifdef ROBOFRIEND_VERSION2
  int LEFT_MOTOR_MINPWM = 1;
  int RIGHT_MOTOR_MINPWM = 1;
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

    // activate the PID controllers
    leftMotorPID.SetMode(AUTOMATIC);
    rightMotorPID.SetMode(AUTOMATIC);

}


void Motor::performPIDControl()
{
  SetpointLeft = (double)intendedLeftSpeed;
  leftMotorPID.Compute();
  leftSpeed= (int)OutputLeft;
  if ((intendedLeftSpeed>=0) && (leftSpeed<0)) leftSpeed=0;
  if ((intendedLeftSpeed<0) && (leftSpeed>0)) leftSpeed=0;


  SetpointRight = (double)intendedRightSpeed;
  rightMotorPID.Compute();
  rightSpeed= (int)OutputRight;
  if ((intendedRightSpeed>=0) && (rightSpeed<0)) rightSpeed=0;
  if ((intendedRightSpeed<0) && (rightSpeed>0)) rightSpeed=0;

  if (intendedDuration>0) {
      intendedDuration--;
      if (!intendedDuration) {
        intendedLeftSpeed=0;
        intendedRightSpeed=0;
    }
  }
}

void Motor::performLinearControl()
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
}
void Motor::performOdomControl()
{
 
  static int loopcounter=0;
/******************** GET START MOTOR PWM */
  if (intendedLeftSpeed > 0 && leftSpeed < leftStartDrive)
  {
    leftSpeed=leftStartDrive;
  }
  if (intendedLeftSpeed < 0 && leftSpeed > leftStartDrive)
  {
    leftSpeed=-leftStartDrive;
  }

  if (intendedRightSpeed > 0 && rightSpeed < rightStartDrive)
  {
    rightSpeed=rightStartDrive;
  }
  if (intendedRightSpeed < 0 && rightSpeed > rightStartDrive)
  {
    rightSpeed=-rightStartDrive;
  }

/************************* anfahren der Motoren */
if(leftSpeedReached==false)
{
  if (intendedLeftSpeed >= leftSpeed+ACCEL_STEP) leftSpeed += ACCEL_STEP;
  else if (intendedLeftSpeed <= leftSpeed-ACCEL_STEP) leftSpeed -= ACCEL_STEP;
  else leftSpeedReached=true;
}

if(rightSpeedReached==false)
{
  if (intendedRightSpeed >= rightSpeed+ACCEL_STEP) rightSpeed += ACCEL_STEP;
  else if (intendedRightSpeed <= rightSpeed-ACCEL_STEP) rightSpeed -= ACCEL_STEP;
  else rightSpeedReached=true;
}
/*************************** Motor reached final speed */
  if (leftSpeedReached && rightSpeedReached && (intendedDuration>0)) {
      intendedDuration--;
      if (!intendedDuration) {
        intendedLeftSpeed=0;
        intendedRightSpeed=0;
    }
  }
//char str[60];
  if (leftSpeedReached){
      
 // sprintf(str,"odo=%d set_left_odom=%d lPWM=%d\n",(int)odo.get_odom_left(), (int)set_left_odom, (int)leftSpeed);
   //   Serial.print(str);
      if (!(loopcounter % 50)) { 
      if(((int)odo.get_odom_left()) > set_left_odom)
      {
        leftSpeed--;
      }
      else if(((int)odo.get_odom_left()) < set_left_odom)
      {
        leftSpeed++;  
      }
      }
  }

  if (rightSpeedReached){
      
  //sprintf(str,"odo=%d set_left_odom=%d lPWM=%d\n",(int)odo.get_odom_right(), (int)set_right_odom, (int)rightSpeed);
    //  Serial.print(str);
      if (!(loopcounter % 50)) { 
      if(((int)odo.get_odom_right()) > set_right_odom)
      {
        rightSpeed--;
      }
      else if(((int)odo.get_odom_right()) < set_right_odom)
      {
        rightSpeed++;  
      }
      }
  }

  
  loopcounter++;
  if (loopcounter > 10000)
    loopcounter = 0;
}



void Motor::updateMotors()
{
  static int cnt=0;
  char str[60];

  cnt++;
  if ((USE_PID_CONTROL==0) && (cnt%5)) return;  // PID control runs 1KHz, linear control 200Hz

  // get left motor encoder timing
  if (odo.getLeftEncoderTime() < MOTOR_STOPPED) 
     InputLeft =  10000.0/(double)odo.getLeftEncoderTime()*odo.getLeftDirection();
  else InputLeft = 0;

  // get right motor encoder timing
  if (odo.getRightEncoderTime() < MOTOR_STOPPED) 
     InputRight =  10000.0/(double)odo.getRightEncoderTime()*odo.getRightDirection();
  else InputRight = 0;

  
  //if (USE_PID_CONTROL) performPIDControl();
  //else performLinearControl();
  performOdomControl();
  
  if (HANDLE_OBSTACLES) handleObstacles();
  
  if (leftSpeed > MOVE_THRESHOLD)  { 
    digitalWrite(PIN_MT_LE_BCK, LOW); digitalWrite(PIN_MT_LE_FWD, HIGH); 
    Timer3.pwm(PIN_MT_LE_PWM, map(leftSpeed,0,255,LEFT_MOTOR_MINPWM,LEFT_MOTOR_MAXPWM));
  }
  else if (leftSpeed < -MOVE_THRESHOLD) { 
    digitalWrite(PIN_MT_LE_FWD, LOW); digitalWrite(PIN_MT_LE_BCK, HIGH); 
    Timer3.pwm(PIN_MT_LE_PWM, map(-leftSpeed,0,255,LEFT_MOTOR_MINPWM,LEFT_MOTOR_MAXPWM));    
  }
  else { digitalWrite(PIN_MT_LE_FWD, LOW);  digitalWrite(PIN_MT_LE_BCK, LOW); Timer3.pwm(PIN_MT_LE_PWM,0);}

  if (rightSpeed > MOVE_THRESHOLD)  { 
    digitalWrite(PIN_MT_RI_BCK, LOW); digitalWrite(PIN_MT_RI_FWD, HIGH); 
    Timer3.pwm(PIN_MT_RI_PWM, map(rightSpeed,0,255,LEFT_MOTOR_MINPWM,RIGHT_MOTOR_MAXPWM));
  }
  else if (rightSpeed < -MOVE_THRESHOLD) { 
    digitalWrite(PIN_MT_RI_FWD, LOW); digitalWrite(PIN_MT_RI_BCK, HIGH); 
    Timer3.pwm(PIN_MT_RI_PWM, map(-rightSpeed,0,255,LEFT_MOTOR_MINPWM,RIGHT_MOTOR_MAXPWM));    
  }
  else { digitalWrite(PIN_MT_RI_FWD, LOW); digitalWrite(PIN_MT_RI_FWD, LOW); Timer3.pwm(PIN_MT_RI_PWM,0);}
  
  if (PRINT_MOTORSPEED_MESSAGES) {

    if (((cnt)%PRINT_MOTORSPEED_MESSAGES == 0) && (leftSpeed || rightSpeed)) {
      sprintf(str,"SPL=%04d IL=%04d lPWM=%03d   ",intendedLeftSpeed, (int)InputLeft, (int)leftSpeed);
      Serial.print(str);
      sprintf(str,"SPR=%04d IR=%04d rPWM=%03d\r\n",intendedRightSpeed, (int)InputRight, (int)rightSpeed);
      Serial.println(str);
    }
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
   //Serial.printf("Drive right=%04d, left=%04d, duration=%04d\n",left,right,duration);
   intendedRightSpeed=right;
   intendedLeftSpeed=left;
  
   if (USE_PID_CONTROL==0)
      intendedDuration=duration;
   else intendedDuration=duration*5;
}

int leftMapping(float odom)
{
    int i =0;
    if(odom >=0 )
    {
        for(i=0;i<150;i++)
        {
            if(motorMapForward[i][1] >= odom)
                break;
        }
        return motorMapForward[i][0];
    }
    else
    {
         for(i=0;i<150;i++)
        {
            if(motorMapBackward[i][1] <= odom)
                break;
        }
        return motorMapBackward[i][0];
    }
    
}

int rightMapping(float odom)
{
    int i =0;
    if(odom >=0 )
    {
        for(i=0;i<150;i++)
        {
            if(motorMapForward[i][2] >= odom)
                break;
        }
        return motorMapForward[i][0];
    }
    else
    {
         for(i=0;i<150;i++)
        {
            if(motorMapBackward[i][2] <= odom)
                break;
        }
        return motorMapBackward[i][0];
    }
}

void Motor::drive_odom(int left, int right, int duration) {
  if(right > MAX_ODOM)
    right = MAX_ODOM;
  if(left > MAX_ODOM)
    left = MAX_ODOM;
  if(right < -MAX_ODOM)
    right = -MAX_ODOM;
  if(left < -MAX_ODOM)
    left = -MAX_ODOM;

  set_left_odom=left;
  set_right_odom=right;
   //Serial.printf("Drive Odom right=%04d, left=%04d, duration=%04d\n",left,right,duration);
   // Serial.printf("Drive Odom Mapping right=%04d, left=%04d, duration=%04d\n",(int)leftMapping(left),(int)rightMapping(right),duration);
  /////////////////////////////// Mapping
   intendedRightSpeed=(int)rightMapping(right);
   intendedLeftSpeed=(int)leftMapping(left);

leftSpeedReached=false;
rightSpeedReached=false;
  
   if (USE_PID_CONTROL==0)
      intendedDuration=duration;
   else intendedDuration=duration*5;
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
