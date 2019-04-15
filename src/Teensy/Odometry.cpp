/*
 * @file    	Odometry.cpp
 * @version 	v10.0
 * @date    	19.11.2018
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

/****************************************************************** INCLUDES */

#include <Arduino.h>
#include <TimerOne.h>
#include "Odometry.h"
#include "GPIO.h"

/******************************************************************* GLOBALS */

int riIncState;
int riLastIncState;
int riDecState;
int riLastDecState;
int leIncState;
int leLastIncState;
int leDecState;
int leLastDecState;
int32_t counterRe;
int32_t counterLe;

int cnt1=0;
int cnt2=0;
int cnt3=0;
int cnt4=0;
uint8_t print_var = 0;
uint8_t newPINB;
uint8_t oldPINB = 0xf0;

/*************************************************************** DEFINITIONS */

Odometry::Odometry() {

}

Odometry::~Odometry() {

}

// TODO: get information when the motor is in idle state

void Odometry::init() {
	PORTB |= 0xF0;
	PCICR |= 0x01;
	PCMSK0 |= 0xF0;
	sei();

	counterRe = 0;
	counterLe = 0;
}

void Odometry::printEncoderValues() {
	char str[100];
	int32_t tmpLe, tmpRe;

	cli();
	tmpLe = counterLe;
	tmpRe = counterRe;
	sei();

	sprintf(str, "left=%05ld right=%05ld", tmpLe, tmpRe);
	Serial.println(str);
}

void Odometry::clearEncoderValues() {
	cli();
	counterLe = counterRe = 0;
	sei();
}

int32_t Odometry::getRightEncoderValue() {
	int32_t tmpRe;
	cli();
	tmpRe = counterRe;
	sei();
	return (tmpRe);
}

int32_t Odometry::getLeftEncoderValue() {
	int32_t tmpLe;
	cli();
	tmpLe = counterLe;
	sei();
	return (tmpLe);
}

ISR(PCINT0_vect) {
	newPINB=PINB;
	if ((newPINB & (1<<7)) != (oldPINB & (1<<7))) counterLe--; // LE DEC
	if ((newPINB & (1<<6)) != (oldPINB & (1<<6))) counterLe++; // LE INC


  if ((newPINB & (1<<6)) != (oldPINB & (1<<6))) {
    if ((newPINB & (1<<6)) != ((newPINB >> 1) & (1<<6)))  
      counterLe++; // RE INC
    else
      counterLe--;
  }
  
	if ((newPINB & (1<<4)) != (oldPINB & (1<<4))) {
	  if ((newPINB & (1<<4)) != ((newPINB >> 1) & (1<<4)))  
	    counterRe--; // RE INC
    else  
      counterRe++;
	}
	oldPINB = newPINB;
}


/**
 * @brief	called from Timer 1 ISR (every millisecond)
 */
void readEncoderValues() {
	Serial.print("*");
//	readEncoderValueRightMotor();
//	readEncoderValueLeftMotor();
}

void readEncoderValueRightMotor() {
	/* read the "current" state of output "right increment" */
	riIncState = digitalRead(PIN_MT_RI_INC);

	/* if the previous and the current state of the output "right increment" are different, that means a pulse has occured */
	if (riIncState != riLastIncState) {
		// If the "right decrement" state is different to the "right increment" state, that means the encoder is rotating clockwise
		riDecState = digitalRead(PIN_MT_RI_DEC);
		if (riDecState != riIncState) {
			counterRe--;
		} else {
			counterRe++;
		}
	}
	riLastIncState = riIncState; // Updates the previous state of the outputA with the current state
}

void readEncoderValueLeftMotor() {
	/* read the "current" state of output "left increment" */
	leIncState = digitalRead(PIN_MT_LE_INC);

	/* if the previous and the current state of the output "left increment" are different, that means a pulse has occured */
	if (leIncState != leLastIncState) {
		// If the "left decrement" state is different to the "left increment" state, that means the encoder is rotating clockwise
		leDecState = digitalRead(PIN_MT_LE_DEC);
		if (leDecState != leIncState) {
			counterLe++;
		} else {
			counterLe--;
		}
	}
	leLastIncState = leIncState; // Updates the previous state of the outputA with the current state
}
