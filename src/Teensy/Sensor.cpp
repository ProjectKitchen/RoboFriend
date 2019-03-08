/*
 * @file    	Parser.h
 * @version 	v10.0
 * @date    	01.01.20xx
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

/****************************************************************** INCLUDES */

#include "Config.h"
#include "GPIO.h"
#include "Motor.h"
#include "Sensor.h"

/******************************************************************* GLOBALS */

volatile bool overCurrentAnalog = false;
volatile bool overCurrentDigital = false;

/******************************************************************* EXTERNS */

extern long loopcounter;

/*************************************************************** DEFINITIONS */

Sensor::Sensor(void) {

}

Sensor::~Sensor(void) {
	if (batteryBuffer)
		free(batteryBuffer);
	if (shuntAmpBuffer)
		free(shuntAmpBuffer);
	if (IRSensorLeftBuffer)
		free(IRSensorLeftBuffer);
	if (IRSensorMiddleBuffer)
		free(IRSensorMiddleBuffer);
	if (IRSensorRightBuffer)
		free(IRSensorRightBuffer);

	pinMode(PIN_OC_DI, INPUT);
	detachInterrupt(digitalPinToInterrupt(PIN_OC_DI));
}

void Sensor::init() {
	ir_lft_thold = Sensor::IR_LFT_THOLD_DEF;
	ir_mid_thold = Sensor::IR_MID_THOLD_DEF;
	ir_ryt_thold = Sensor::IR_RYT_THOLD_DEF;

	/* create circular buffers */
	batteryBuffer = new RunningAverage(Sensor::BAT_BUFFER_SIZE);
	shuntAmpBuffer = new RunningAverage(Sensor::SHUNT_AMP_BUFFER_SIZE);
	IRSensorLeftBuffer = new RunningAverage(Sensor::IR_BUFFER_SIZE);
	IRSensorMiddleBuffer = new RunningAverage(Sensor::IR_BUFFER_SIZE);
	IRSensorRightBuffer = new RunningAverage(Sensor::IR_BUFFER_SIZE);

	pinMode(PIN_OC_DI, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PIN_OC_DI), readComparatorValue, CHANGE);
}

void Sensor::readSensorValues() {
	batteryBuffer->addValue(analogRead(PIN_ADC_VBAT));
	shuntAmpBuffer->addValue(analogRead(PIN_OC_AN));
	IRSensorLeftBuffer->addValue(analogRead(PIN_ADC_IR1));
	IRSensorMiddleBuffer->addValue(analogRead(PIN_ADC_IR2));
	IRSensorRightBuffer->addValue(analogRead(PIN_ADC_IR3));

	battery = batteryBuffer->getFastAverage();
	shuntAmp = shuntAmpBuffer->getFastAverage();
	ir_lft = IRSensorLeftBuffer->getFastAverage();
	ir_mid = IRSensorMiddleBuffer->getFastAverage();
	ir_ryt = IRSensorRightBuffer->getFastAverage();

	ir_lft_trig = (ir_lft >= ir_lft_thold);
	ir_mid_trig = (ir_mid >= ir_mid_thold);
	ir_ryt_trig = (ir_ryt >= ir_ryt_thold);

	float shuntAmpVoltage = Sensor::ADC_INTERNAL_VREF / (float)Sensor::ADC_RESOLUTION * shuntAmp;
	float maxVoltage = Motor::MOTORS_MAX_CURRENT * Sensor::SHUNT_AMP_MAX_VOLTAGE / Sensor::SHUNT_AMP_MAX_CURRENT;
	if (shuntAmpVoltage >= maxVoltage) {
		overCurrentAnalog = true;
		// TODO: handle overcurrent
	} else {
		overCurrentAnalog = false;
		// TODO: clear overcurrent
	}

#if IMU
	byte num=0;

	Wire.beginTransmission(IMU_ADRESS);
	Wire.write(0); // addresss high byte
	Wire.write(0); // address low byte
	Wire.endTransmission();

	// read 1 byte
	Wire.requestFrom(IMU_ADRESS, 1);
	while(Wire.read()) {
		num = Wire.receive();
	}
	Serial.print("num = ");
	Serial.println(num, DEC);
#endif

}

void Sensor::provideSensorValues() {
	Serial.printf("Sensors,%04d,%04d,%04d,%04d\n",
			battery, ir_lft, ir_mid, ir_ryt);
}

void Sensor::setSensorThresholds(int left, int middle, int right) {
	ir_lft_thold = left;
	ir_mid_thold = middle;
	ir_ryt_thold = right;
}

int Sensor::getIRSensorLeftValue() {
	return ir_lft;
}

int Sensor::getIRSensorMiddleValue() {
	return ir_mid;
}

int Sensor::getIRSensorRightValue() {
	return ir_ryt;
}

bool Sensor::isIRSensorLeftTriggered() {
	return ir_lft_trig;
}

bool Sensor::isIRSensorMiddleTriggered() {
	return ir_mid_trig;
}

bool Sensor::isIRSensorRightTriggered() {
	return ir_ryt_trig;
}

void readComparatorValue() {
	overCurrentDigital = !overCurrentDigital;
	// TODO: handle overcurrent
}

