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

// #define TEST_ENCODERS

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
int16_t timeRe=0;
int16_t timeLe=0;


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
  static int16_t actTimeRe=0;
  /* read the "current" state of output "right increment" */
  riIncState = digitalRead(PIN_MT_RI_INC);
  actTimeRe++;  
  if (actTimeRe>1000) {actTimeRe=0;timeRe=1000;}
  /* if the previous and the current state of the output "right increment" are different, that means a pulse has occured */
  if (riIncState != riLastIncState) {  
      // If the "right decrement" state is different to the "right increment" state, that means the encoder is rotating clockwise
     timeRe=actTimeRe; 
     actTimeRe=0;
     riDecState = digitalRead(PIN_MT_RI_DEC);
     if (riDecState != riIncState) { 
       counterRe--;
     } else {
       counterRe++;
     }
   } 
   riLastIncState = riIncState; // Updates the previous state of the outputA with the current state
}

/**
 * @brief
 */
void getStateLeftMotor() {
  static int16_t actTimeLe=0;
  /* read the "current" state of output "left increment" */
  leIncState = digitalRead(PIN_MT_LE_INC);
  actTimeLe++;
  if (actTimeLe>1000) {actTimeLe=0;timeLe=1000;}
  /* if the previous and the current state of the output "left increment" are different, that means a pulse has occured */
  if (leIncState != leLastIncState) {   
     // If the "left decrement" state is different to the "left increment" state, that means the encoder is rotating clockwise
     timeLe=actTimeLe; 
     actTimeLe=0;
     leDecState = digitalRead(PIN_MT_LE_DEC);
     if (leDecState != leIncState) { 
       counterLe ++;
     } else {
       counterLe --;
     }
   } 
   leLastIncState = leIncState; // Updates the previous state of the outputA with the current state
}


void Odometry::printEncoderValues() {
    char str[100];
    int32_t tmpCounterLe, tmpCounterRe;
    int16_t tmpTimeLe, tmpTimeRe;
    
    cli();
    tmpCounterLe=counterLe;
    tmpCounterRe=counterRe;
    tmpTimeLe=timeLe;
    tmpTimeRe=timeRe;
    sei();
    
    //sprintf(str,"left=%05ld right=%05ld",tmpCounterLe,tmpCounterRe);
    sprintf(str,"leftTime=%05d rightTime=%05d",tmpTimeLe,tmpTimeRe);
    Serial.println(str);
}

void Odometry::clearEncoderValues() {
    cli();
    counterLe=counterRe=0;
    sei();
}

int32_t Odometry::getRightEncoderValue() {
    int32_t tmpRe;
    cli();
    tmpRe=counterRe;
    sei();
    return(tmpRe);    
}

int32_t Odometry::getLeftEncoderValue() {
    int32_t tmpLe;
    cli();
    tmpLe=counterLe;
    sei();
    return(tmpLe);    
}


int16_t Odometry::getRightEncoderTime() {
    int16_t tmpTimeRe;
    cli();
    tmpTimeRe=timeRe;
    sei();
    return(tmpTimeRe);    
}

int16_t Odometry::getLeftEncoderTime() {
    int16_t tmpTimeLe;
    cli();
    tmpTimeLe=timeLe;
    sei();
    return(tmpTimeLe);    
}
