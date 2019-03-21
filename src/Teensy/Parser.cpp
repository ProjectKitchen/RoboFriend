/*
 * @file    	Parser.cpp
 * @version 	v10.0
 * @date    	01.01.20xx
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

/****************************************************************** INCLUDES */

#include <Arduino.h>
#include "Parser.h"
#include "Motor.h"
#include "Sensor.h"
#include "SerialCommand.h"

/*************************************************************** DEFINITIONS */

SerialCommand sc;

Parser::Parser() {

}

Parser::~Parser() {

}

void Parser::init() {
	sc.addCommand("D", drive);
	sc.addCommand("R", provideSensorValues);
	sc.addCommand("S", setSensorThresholds);
	sc.addDefaultHandler(defaultHandler);
}

void Parser::processSerialCommands() {
	sc.readSerial();
}

void drive() {
	char *arg;
	int left = 0;
	int right = 0;
	int duration = -1;

	arg = sc.next();
	if (arg != NULL) {
		left = atoi(arg);
	}
	arg = sc.next();
	if (arg != NULL) {
		right = atoi(arg);
	}
	arg = sc.next();
	if (arg != NULL) {
		duration = atoi(arg);
	}
	motors.setIntendedParam(left, right, duration);
}

void provideSensorValues() {
	sensors.provideSensorValues();
}

void setSensorThresholds() {
	char *arg;
	int left = Sensor::IR_LFT_THOLD_DEF;
	int middle = Sensor::IR_MID_THOLD_DEF;
	int right =Sensor:: IR_RYT_THOLD_DEF;

	arg = sc.next();
	if (arg != NULL) {
		left = atoi(arg);
	}
	arg = sc.next();
	if (arg != NULL) {
		middle = atoi(arg);
	}
	arg = sc.next();
	if (arg != NULL) {
		right = atoi(arg);
	}
	sensors.setSensorThresholds(left, middle, right);
}

void defaultHandler() {
	char *arg;
	arg = sc.next();
	Serial.printf("Received unsupported command: %c\n", arg); // TODO: print argument
}

