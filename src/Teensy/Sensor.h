/*
 * @file    	Parser.h
 * @version 	v10.0
 * @date    	01.01.20xx
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

#ifndef SENSOR_H_
#define SENSOR_H_

/****************************************************************** INCLUDES */

#include <Wire.h>
#include "RunningAverage.h"   // library used for averaging sensor data

/******************************************************************* EXTERNS */

extern class Sensor sensors;

/********************************************************** CLASS DEFINITION */

class Sensor {
public:
	static const uint8_t BAT_BUFFER_SIZE = 80;
	static const uint8_t IR_BUFFER_SIZE = 10;
	static const uint16_t IR_LFT_THOLD_DEF = 220;
	static const uint16_t IR_MID_THOLD_DEF = 450;
	static const uint16_t IR_RYT_THOLD_DEF = 220;

	Sensor(void);
	~Sensor(void);

	void init(void);
	void readSensorValues(void);
	void provideSensorValues(void);
	void setSensorThresholds(int left, int middle, int right);

	int getIRSensorLeftValue(void);
	int getIRSensorMiddleValue(void);
	int getIRSensorRightValue(void);

	bool isIRSensorLeftTriggered(void);
	bool isIRSensorMiddleTriggered(void);
	bool isIRSensorRightTriggered(void);

private:
	int battery;

	int ir_lft;
	int ir_lft_thold;
	bool ir_lft_trig;

	int ir_mid;
	int ir_mid_thold;
	bool ir_mid_trig;

	int ir_ryt;
	int ir_ryt_thold;
	bool ir_ryt_trig;

	RunningAverage *batteryBuffer;
	RunningAverage *IRSensorLeftBuffer;
	RunningAverage *IRSensorMiddleBuffer;
	RunningAverage *IRSensorRightBuffer;
};

#endif /* SENSOR_H_ */
