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
	static const uint8_t SHUNT_AMP_BUFFER_SIZE = 50;
	static const uint8_t IR_BUFFER_SIZE = 10;
	static const uint16_t IR_LFT_THOLD_DEF = 220;
	static const uint16_t IR_MID_THOLD_DEF = 450;
	static const uint16_t IR_RYT_THOLD_DEF = 220;

	/**
	 * over current measurement with INA193 48V 10A current shunt amplifier,
	 * also see: https://www.tinacloud.com/tinademo/tina.php?path=EXAMPLESROOT%7CUSER%7C&file=48V%2010A%20Current%20Measurement.TSC
	 *
	 * the shunt amp has a linear output, 10A are represented by 2V at the output
	 * we choose to define 5A as the max. current consumed by both motors.
	 * since we have a linear output, the 5A will be represented by 1V at the output
	 */
	static const uint8_t ADC_INTERNAL_VREF = 5;
	static const float ADC_EXTERNAL_VREG = 4.096;
	static const uint16_t ADC_RESOLUTION = 1023;
	static const uint8_t SHUNT_AMP_MAX_CURRENT = 10; // [A]
	static const uint8_t SHUNT_AMP_MAX_VOLTAGE = 2; // [V]

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
	int shuntAmp;
	bool overCurrent;

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
	RunningAverage *shuntAmpBuffer;
	RunningAverage *IRSensorLeftBuffer;
	RunningAverage *IRSensorMiddleBuffer;
	RunningAverage *IRSensorRightBuffer;
};

#endif /* SENSOR_H_ */
