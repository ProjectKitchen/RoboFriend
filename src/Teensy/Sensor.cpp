
#include "Sensor.h"

Sensor::Sensor() {
}

Sensor::~Sensor() {
  if (IRSensorLeftBuffer) free(IRSensorLeftBuffer);
  if (IRSensorMiddleBuffer) free(IRSensorMiddleBuffer);
  if (IRSensorRightBuffer) free(IRSensorRightBuffer);
}

void Sensor::init () {
  // Circular Buffers for IR sensor data
  IRSensorLeftBuffer=new RunningAverage(IR_SENSOR_AVERAGER_SIZE);
  IRSensorMiddleBuffer=new RunningAverage(IR_SENSOR_AVERAGER_SIZE);
  IRSensorRightBuffer=new RunningAverage(IR_SENSOR_AVERAGER_SIZE);
}

void Sensor::updateSensorData()
{
    Battery = analogRead(BATTERY_SENSOR_PIN);
    
    IRSensorLeftBuffer->addValue(analogRead(IR_SENSOR_LEFT_PIN));
    IRSensorMiddleBuffer->addValue(analogRead(IR_SENSOR_MIDDLE_PIN));
    IRSensorRightBuffer->addValue(analogRead(IR_SENSOR_RIGHT_PIN));

    IRSensorMiddle = IRSensorMiddleBuffer->getFastAverage();
    IRSensorLeft   = IRSensorLeftBuffer->getFastAverage();
    IRSensorRight  = IRSensorRightBuffer->getFastAverage();

    IRSensorMiddleTriggered = (IRSensorMiddle >= IRSensorMiddleThreshold);
    IRSensorLeftTriggered   = (IRSensorLeft >= IRSensorLeftThreshold);
    IRSensorRightTriggered  = (IRSensorRight >= IRSensorRightThreshold);
}

void Sensor::reportSensorValues() {

    // send sensor status
    Serial.println(Battery);
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

