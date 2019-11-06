
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

  ircam.init_ir();
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

    result = ircam.read_ir();
    //if ((loopcounter % 200) == 0) reportSensorValues();
}

void Sensor::reportSensorValues() {

    // send sensor status
    Serial.printf("Sensors,%04d,%04d,%04d,%04d\n",Battery,IRSensorLeft,IRSensorMiddle,IRSensorRight);
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

void Sensor::readIrToken(){

if(ircam.token1.valid)
{
  Serial.printf("IRToken,1,%04d,%04d,%04d",ircam.token1.x,ircam.token1.y,ircam.token1.tokenSize);
}
else
{
  Serial.printf("IRToken,1,-1,-1,-1");
}

if(ircam.token2.valid)
{
  Serial.printf(",2,%04d,%04d,%04d",ircam.token2.x,ircam.token2.y,ircam.token2.tokenSize);
}
else
{
  Serial.printf(",2,-1,-1,-1");
}

if(ircam.token3.valid)
{
  Serial.printf(",3,%04d,%04d,%04d",ircam.token3.x,ircam.token3.y,ircam.token3.tokenSize);
}
else
{
  Serial.printf(",3,-1,-1,-1");
}


if(ircam.token4.valid)
{
  Serial.printf(",4,%04d,%04d,%04d\n",ircam.token4.x,ircam.token4.y,ircam.token4.tokenSize);
}
else
{
  Serial.printf(",4,-1,-1,-1\n");
}
}
