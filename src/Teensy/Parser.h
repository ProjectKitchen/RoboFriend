/*
 * @file    	Parser.h
 * @version 	v10.0
 * @date    	01.01.20xx
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

#ifndef PARSER_H_
#define PARSER_H_

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
