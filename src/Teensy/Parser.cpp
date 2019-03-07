/*
 * @file    	Parser.cpp
 * @version 	v1.0
 * @date    	07.03.2019
 * @changed 	07.03.2019
 * @author  	mzahedi
 * @brief   	DESCRIPTION
 */

/****************************************************************** INCLUDES */

#include "Parser.h"

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
	motors.drive(left, right, duration);
}

void provideSensorValues() {
	sensors.reportSensorValues();
}

void setSensorThresholds() {
	char *arg;
	int left = DEFAULT_IR_LEFT_THRESHOLD;
	int middle = DEFAULT_IR_MIDDLE_THRESHOLD;
	int right = DEFAULT_IR_RIGHT_THRESHOLD;

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
	Serial.printf("Received unsupported command: %c\n", arg);
}

