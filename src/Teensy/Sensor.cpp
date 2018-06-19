
#include "Sensor.h"

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
    BatterySensorBuffer->addValue(analogRead(BATTERY_SENSOR_PIN));
    
    IRSensorMiddleBuffer->addValue(analogRead(IR_SENSOR_MIDDLE_PIN));
    IRSensorLeftBuffer->addValue(analogRead(IR_SENSOR_LEFT_PIN));
    IRSensorRightBuffer->addValue(analogRead(IR_SENSOR_RIGHT_PIN));

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
    Serial.printf("Sensors,%04d,%04d,%04d,%04d\n",Battery,IRSensorLeft,IRSensorMiddle,IRSensorRight);
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

