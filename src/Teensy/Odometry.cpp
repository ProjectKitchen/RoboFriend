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

/*************************************************************** DEFINITIONS */

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
  Serial.println("Odometry setup");
}

/**
 * @brief
 */
void Odometry::getStateRightMotor() {
  /* read the "current" state of output "right increment" */
  riIncState = digitalRead(PIN_MT_RI_INC);
  
  /* if the previous and the current state of the output "right increment" are different, that means a pulse has occured */
  if (riIncState != riLastIncState) {   
     // If the "right decrement" state is different to the "right increment" state, that means the encoder is rotating clockwise
     riDecState = digitalRead(PIN_MT_RI_DEC);
     if (riDecState != riIncState) { 
       counterRe ++;
     } else {
       counterRe --;
     }
     Serial.print("Position Ri: ");
     Serial.println(counterRe);
   } 
   riLastIncState = riIncState; // Updates the previous state of the outputA with the current state
}

/**
 * @brief
 */
void Odometry::getStateLeftMotor() {
  /* read the "current" state of output "left increment" */
  leIncState = digitalRead(PIN_MT_LE_INC);
  
  /* if the previous and the current state of the output "left increment" are different, that means a pulse has occured */
  if (leIncState != leLastIncState) {   
     // If the "left decrement" state is different to the "left increment" state, that means the encoder is rotating clockwise
     leDecState = digitalRead(PIN_MT_LE_DEC);
     if (leDecState != leIncState) { 
       counterLe ++;
     } else {
       counterLe --;
     }
     Serial.print("Position Le: ");
     Serial.println(counterLe);
   } 
   leLastIncState = leIncState; // Updates the previous state of the outputA with the current state
}

