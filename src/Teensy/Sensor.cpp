#include "Sensor.h"
#include "GPIO.h"

extern long loopcounter;

Sensor::Sensor(void) {
	  ir_lft_thold = Sensor::IR_LFT_THOLD_DEF;
	  ir_mid_thold = Sensor::IR_MID_THOLD_DEF;
	  ir_ryt_thold = Sensor::IR_RYT_THOLD_DEF;
}

Sensor::~Sensor(void) {
	if (batteryBuffer)
		free(batteryBuffer);
	if (IRSensorLeftBuffer)
		free(IRSensorLeftBuffer);
	if (IRSensorMiddleBuffer)
		free(IRSensorMiddleBuffer);
	if (IRSensorRightBuffer)
		free(IRSensorRightBuffer);
}

void Sensor::init() {
	/* create circular buffers */
	batteryBuffer = new RunningAverage(Sensor::BAT_BUFFER_SIZE);
	IRSensorLeftBuffer = new RunningAverage(Sensor::IR_BUFFER_SIZE);
	IRSensorMiddleBuffer = new RunningAverage(Sensor::IR_BUFFER_SIZE);
	IRSensorRightBuffer = new RunningAverage(Sensor::IR_BUFFER_SIZE);
}

void Sensor::updateSensorData() {
	batteryBuffer->addValue(analogRead(PIN_ADC_VBAT));

	IRSensorLeftBuffer->addValue(analogRead(PIN_ADC_IR1));
	IRSensorMiddleBuffer->addValue(analogRead(PIN_ADC_IR2));
	IRSensorRightBuffer->addValue(analogRead(PIN_ADC_IR3));

	battery = batteryBuffer->getFastAverage();
	ir_lft = IRSensorLeftBuffer->getFastAverage();
	ir_mid = IRSensorMiddleBuffer->getFastAverage();
	ir_ryt = IRSensorRightBuffer->getFastAverage();

	ir_lft_trig = (ir_lft >= ir_lft_thold);
	ir_mid_trig = (ir_mid >= ir_mid_thold);
	ir_ryt_trig = (ir_ryt >= ir_ryt_thold);

	//if ((loopcounter % 200) == 0) reportSensorValues();
}

void Sensor::provideSensorValues() {
	Serial.printf("Sensors,%04d,%04d,%04d,%04d\n", battery, ir_lft, ir_mid, ir_ryt);
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

