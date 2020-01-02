/*
 * @file    Odometry.cpp
 * @version v1.0
 * @date    19.11.2018
 * @author  mzahedi
 * @brief   DESCRIPTION
 */

/****************************************************************** INCLUDES */

#include <Arduino.h>
#include "Odometry.h"
#include "support.h"
#include "GPIO.h"
#include <TimerOne.h>

/*************************************************************** DEFINITIONS */

#define ENCODER_AVG_LEN 5
// #define TEST_ENCODERS


#define DISABLE_TIMER1_OVERFLOW_INT TIMSK1 &= ~(1<<TOIE1)
#define ENABLE_TIMER1_OVERFLOW_INT  TIMSK1 |=  (1<<TOIE1)

int riIncState;
int riLastIncState;    
int riDecState;
int riLastDecState;
int leIncState;
int leLastIncState;
int leDecState;
int leLastDecState;
int32_t counterRe=0;
int32_t counterLe=0;
uint16_t timeRe=0;
uint16_t timeLe=0;
int8_t dirLe=0;
int8_t dirRe=0;


/**
 * @brief
 */
Odometry::Odometry() {

}

/**
 * @brief
 */
Odometry::~Odometry() {
  
}

void updateEncoders()   // called from Timer 1 ISR (200 uS)
{
  getStateRightMotor();
  getStateLeftMotor();

}


uint16_t avgLeft(uint16_t acttime)
{
  static uint16_t buf[ENCODER_AVG_LEN] = {0};
  static uint16_t actpos=0;
  static uint16_t sum=0;

  buf[actpos]=acttime;
  sum+=acttime;
  actpos=(actpos+1) % ENCODER_AVG_LEN;
  sum-=buf[actpos];
  return(sum/ENCODER_AVG_LEN);
}

uint16_t avgRight(uint16_t acttime)
{
  static uint16_t buf[ENCODER_AVG_LEN] = {0};
  static uint16_t actpos=0;
  static uint16_t sum=0;

  buf[actpos]=acttime;
  sum+=acttime;
  actpos=(actpos+1) % ENCODER_AVG_LEN;
  sum-=buf[actpos];
  return(sum/ENCODER_AVG_LEN);
}


void Odometry::init() {
  /* set pin mode for right motor's increment and decrement signals */
  pinMode(PIN_MT_RI_INC, INPUT);
  pinMode(PIN_MT_RI_DEC, INPUT);
  /* set pin mode for left motor's increment and decrement signals */
  pinMode(PIN_MT_LE_INC, INPUT);
  pinMode(PIN_MT_LE_DEC, INPUT);
  /* read the initial state of outputs */
  riLastIncState = digitalRead(PIN_MT_RI_INC);
  riLastDecState = digitalRead(PIN_MT_RI_DEC);
  leLastIncState = digitalRead(PIN_MT_LE_INC);
  leLastDecState = digitalRead(PIN_MT_LE_DEC);

  counterRe = 0;
  counterLe = 0;
  timeRe=0;
  timeLe=0;
  dirLe=0;
  dirRe=0;
  
  //  Serial.println("Odometry setup");

  #ifdef TEST_ENCODERS
    char str[100];
    while (1) {
      sprintf(str,"RInc=%d, RDec=%d, LInc=%d, LDec=%d\n",digitalRead(PIN_MT_RI_INC),digitalRead(PIN_MT_RI_DEC),
                   digitalRead(PIN_MT_LE_INC),digitalRead(PIN_MT_LE_DEC));
      Serial.print(str);
      delay(200);
    }
  #endif

  Timer1.initialize(200);   // 200us = 5kHz !
  Timer1.attachInterrupt(updateEncoders); // blinkLED to run every 0.15 seconds
}

/**
 * @brief
 */
void getStateRightMotor() {
  static uint16_t actTimeRe=0;
  /* read the "current" state of output "right increment" */
  riIncState = digitalRead(PIN_MT_RI_INC);
  actTimeRe+=4;         // add 4 to improve averaging behaviour with integers for PID control!
  if (actTimeRe>1000) {actTimeRe=0;timeRe=MOTOR_STOPPED;}
  /* if the previous and the current state of the output "right increment" are different, that means a pulse has occured */
  if (riIncState != riLastIncState) {  

     timeRe=avgRight(actTimeRe); 
     actTimeRe=0;

      // If the "right decrement" state is different to the "right increment" state, that means the encoder is rotating clockwise
     riDecState = digitalRead(PIN_MT_RI_DEC);
     if (riDecState != riIncState) { 
       counterRe--; dirRe=-1;
     } else {
       counterRe++; dirRe=1;
     }
   } 
   riLastIncState = riIncState; // Updates the previous state of the outputA with the current state
}

/**
 * @brief
 */
void getStateLeftMotor() {
  static uint16_t actTimeLe=0;
  /* read the "current" state of output "left increment" */
  leIncState = digitalRead(PIN_MT_LE_INC);
  actTimeLe+=4;       // add 4 to improve averaging behaviour with integers for PID control!
  if (actTimeLe>1000) {actTimeLe=0;timeLe=MOTOR_STOPPED;}
  /* if the previous and the current state of the output "left increment" are different, that means a pulse has occured */
  if (leIncState != leLastIncState) {   

     timeLe=avgLeft(actTimeLe); 
     actTimeLe=0;

     // If the "left decrement" state is different to the "left increment" state, that means the encoder is rotating clockwise
     leDecState = digitalRead(PIN_MT_LE_DEC);
     if (leDecState != leIncState) { 
       counterLe++; dirLe=1;
     } else {
       counterLe--; dirLe=-1;
     }
   } 
   leLastIncState = leIncState; // Updates the previous state of the outputA with the current state
}


void Odometry::printEncoderValues() {
    char str[100];
    int32_t tmpCounterLe, tmpCounterRe;
    uint16_t tmpTimeLe, tmpTimeRe;
    
    DISABLE_TIMER1_OVERFLOW_INT;
    tmpCounterLe=counterLe;
    tmpCounterRe=counterRe;
    tmpTimeLe=timeLe;
    tmpTimeRe=timeRe;
    ENABLE_TIMER1_OVERFLOW_INT;
    
    sprintf(str,"left=%05ld right=%05ld",tmpCounterLe,tmpCounterRe);
    //sprintf(str,"leftTime=%05d rightTime=%05d",tmpTimeLe,tmpTimeRe);
    Serial.println(str);
}

void Odometry::clearEncoderValues() {
    DISABLE_TIMER1_OVERFLOW_INT;
    counterLe=counterRe=0;
    ENABLE_TIMER1_OVERFLOW_INT;
}

int32_t Odometry::getRightEncoderValue() {
    int32_t tmpRe;
    DISABLE_TIMER1_OVERFLOW_INT;
    tmpRe=counterRe;
    ENABLE_TIMER1_OVERFLOW_INT;
    return(tmpRe);    
}

int32_t Odometry::getLeftEncoderValue() {
    int32_t tmpLe;
    DISABLE_TIMER1_OVERFLOW_INT;
    tmpLe=counterLe;
    ENABLE_TIMER1_OVERFLOW_INT;
    return(tmpLe);    
}


uint16_t Odometry::getRightEncoderTime() {
    uint16_t tmpTimeRe;
    DISABLE_TIMER1_OVERFLOW_INT;
    tmpTimeRe=timeRe;
    ENABLE_TIMER1_OVERFLOW_INT;
    return(tmpTimeRe);    
}

uint16_t Odometry::getLeftEncoderTime() {
    uint16_t tmpTimeLe;
    DISABLE_TIMER1_OVERFLOW_INT;
    tmpTimeLe=timeLe;
    ENABLE_TIMER1_OVERFLOW_INT;
    return(tmpTimeLe);    
}

int8_t Odometry::getRightDirection() {
    int8_t tmpDirRe;
    DISABLE_TIMER1_OVERFLOW_INT;
    tmpDirRe=dirRe;
    ENABLE_TIMER1_OVERFLOW_INT;
    return(tmpDirRe);    
}

int8_t Odometry::getLeftDirection() {
    int8_t tmpDirLe;
    DISABLE_TIMER1_OVERFLOW_INT;
    tmpDirLe=dirLe;
    ENABLE_TIMER1_OVERFLOW_INT;
    return(tmpDirLe);    
}



  double odom_left;
  double odom_right;

void Odometry::updateOdometry(){
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();
  int32_t tmpCounterLe, tmpCounterRe;

  DISABLE_TIMER1_OVERFLOW_INT;
    tmpCounterLe=counterLe;
    tmpCounterRe=counterRe;
    ENABLE_TIMER1_OVERFLOW_INT;

    clearEncoderValues();
  odom_left = MILLIMETER_PER_ENCODER_STEP * tmpCounterLe*dirLe / ((newTime-oldTime));
  odom_right = MILLIMETER_PER_ENCODER_STEP * tmpCounterRe*dirRe / ((newTime-oldTime));
  oldTime=millis();
  
}

//https://forum.arduino.cc/index.php?topic=44216.0
void printDouble( double val, byte precision){
 // prints val with number of decimal places determine by precision
 // precision is a number from 0 to 6 indicating the desired decimial places
 // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

 Serial.print (int(val));  //prints the int part
 if( precision > 0) {
   Serial.print("."); // print the decimal point
   unsigned long frac;
   unsigned long mult = 1;
   byte padding = precision -1;
   while(precision--)
      mult *=10;
     
   if(val >= 0)
     frac = (val - int(val)) * mult;
   else
     frac = (int(val)- val ) * mult;
   unsigned long frac1 = frac;
   while( frac1 /= 10 )
     padding--;
   while(  padding--)
     Serial.print("0");
   Serial.print(frac,DEC) ;
 }
}

double Odometry::get_odom_left() {
  return odom_left;
}

double Odometry::get_odom_right() {
  return odom_right;
}

void Odometry::printOdom() {
  char str[100];
 /* sprintf(str,"left=%d.%d right=%d.%d",int(odom_left),int(odom_left-int(odom_left)*1000),int(odom_right),int(odom_right-int(odom_right)*1000));
    Serial.println(str);
    */
    Serial.print("left=");
    if(dirLe==-1)
       Serial.print("-");
    printDouble(odom_left,2);
    Serial.print("right=");
        if(dirRe==-1)
       Serial.print("-");
    printDouble(odom_right,2);
    Serial.print("\n");
}
