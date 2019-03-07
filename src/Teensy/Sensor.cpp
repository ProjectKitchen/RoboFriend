
#include "Sensor.h"
#include "GPIO.h"

extern long loopcounter;

Sensor::Sensor() {

}

Sensor::~Sensor() {
  if (IRSensorLeftBuffer) free(IRSensorLeftBuffer);
  if (IRSensorMiddleBuffer) free(IRSensorMiddleBuffer);
  if (IRSensorRightBuffer) free(IRSensorRightBuffer);
}

void Sensor::init () {
  // create circular buffers
  BatterySensorBuffer=new RunningAverage(BATTERY_SENSOR_AVERAGER_SIZE);
  IRSensorLeftBuffer=new RunningAverage(IR_SENSOR_AVERAGER_SIZE);
  IRSensorMiddleBuffer=new RunningAverage(IR_SENSOR_AVERAGER_SIZE);
  IRSensorRightBuffer=new RunningAverage(IR_SENSOR_AVERAGER_SIZE);
}

void Sensor::updateSensorData()
{
    BatterySensorBuffer->addValue(analogRead(PIN_ADC_VBAT));
    
    IRSensorMiddleBuffer->addValue(analogRead(PIN_ADC_IR2));
    IRSensorLeftBuffer->addValue(analogRead(PIN_ADC_IR1));
    IRSensorRightBuffer->addValue(analogRead(PIN_ADC_IR3));

    Battery        = BatterySensorBuffer->getFastAverage();
    IRSensorMiddle = IRSensorMiddleBuffer->getFastAverage();
    IRSensorLeft   = IRSensorLeftBuffer->getFastAverage();
    IRSensorRight  = IRSensorRightBuffer->getFastAverage();

    IRSensorMiddleTriggered = (IRSensorMiddle >= IRSensorMiddleThreshold);
    IRSensorLeftTriggered   = (IRSensorLeft >= IRSensorLeftThreshold);
    IRSensorRightTriggered  = (IRSensorRight >= IRSensorRightThreshold);

    //if ((loopcounter % 200) == 0) reportSensorValues();
}

void Sensor::reportSensorValues() {
    // send sensor status
    Serial.printf("Sensors,%04d,%04d,%04d,%04d\n",
    		Battery, IRSensorLeft, IRSensorMiddle, IRSensorRight);
}

void Sensor::setSensorThresholds(int left, int middle, int right) {
    IRSensorLeftThreshold=left;
    IRSensorMiddleThreshold=middle;
    IRSensorRightThreshold=right;
}


int Sensor::getIRSensorLeftValue() {
  return IRSensorLeft;
}

int Sensor::getIRSensorMiddleValue() {
  return IRSensorMiddle;
}

int Sensor::getIRSensorRightValue() {
  return IRSensorRight;
}

bool Sensor::isIRSensorLeftTriggered() {
  return IRSensorLeftTriggered;
}

bool Sensor::isIRSensorMiddleTriggered() {
  return IRSensorMiddleTriggered;
}

bool Sensor::isIRSensorRightTriggered() {
  return IRSensorRightTriggered;
}

