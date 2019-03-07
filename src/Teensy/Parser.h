/*
 * @file    	Parser.h
 * @version 	v1.0
 * @date    	07.03.2019
 * @changed 	07.03.2019
 * @author  	mzahedi
 * @brief   	DESCRIPTION
 */

#ifndef PARSER_H_
#define PARSER_H_

/****************************************************************** INCLUDES */

#include <Arduino.h>
#include "Motor.h"
#include "Sensor.h"
#include "SerialCommand.h"

/********************************************************** CLASS DEFINITION */

class Parser {
public:
    Parser();
    ~Parser();

    void init(void);
    void processSerialCommands(void);    
};

  void drive();
  void provideSensorValues();
  void setSensorThresholds();
  void defaultHandler();

#endif /* PARSER_H_ */
