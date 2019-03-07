#ifndef SENSOR_H_
#define SENSOR_H_

#include "RunningAverage.h"   // Library used for averaging sensor data

#define IR_SENSOR_AVERAGER_SIZE  10
#define BATTERY_SENSOR_AVERAGER_SIZE  80
#define DEFAULT_IR_LEFT_THRESHOLD   220
#define DEFAULT_IR_MIDDLE_THRESHOLD 450
#define DEFAULT_IR_RIGHT_THRESHOLD  220

extern class Sensor sensors;

class Sensor {

public:
  Sensor(void);
  ~Sensor();

  void init();
  void updateSensorData();
  void reportSensorValues();
  void setSensorThresholds(int left, int middle, int right);

  int getBatteryValue();
  int getIRSensorLeftValue();
  int getIRSensorMiddleValue();
  int getIRSensorRightValue();

  bool isIRSensorLeftTriggered();
  bool isIRSensorMiddleTriggered();
  bool isIRSensorRightTriggered();

protected:
  int Battery = 0; 

  int IRSensorMiddleThreshold = DEFAULT_IR_MIDDLE_THRESHOLD;
  int IRSensorLeftThreshold   = DEFAULT_IR_LEFT_THRESHOLD;
  int IRSensorRightThreshold  = DEFAULT_IR_RIGHT_THRESHOLD;

  int IRSensorMiddle = 0;
  int IRSensorLeft = 0;
  int IRSensorRight = 0;

  bool IRSensorMiddleTriggered;
  bool IRSensorLeftTriggered;
  bool IRSensorRightTriggered;

  RunningAverage * BatterySensorBuffer;
  RunningAverage * IRSensorLeftBuffer;
  RunningAverage * IRSensorMiddleBuffer;
  RunningAverage * IRSensorRightBuffer;
};

#endif /* SENSOR_H_ */





