/*
   @file      Teensy.ino
   @version   v10.0
   @date      01.01.20xx
   @changed   07.03.2019
   @author    cveigl, mzahedi
   @brief     Robofriend Teensy++2.0 firmware
              Implements PWM motor control, sensor management and serial communication with Raspberry Pi
*/

/****************************************************************** INCLUDES */

#include <avr/wdt.h>
#include "Motor.h"
#include "Sensor.h"
#include "Parser.h"
#include "Odometry.h"
#include "GPIO.h"
#include "Config.h"

/******************************************************************* GLOBALS */

Sensor sensors;
Motor motors;
Odometry odo;
Parser parser;

long loopcounter = 0;
long timestamp;

/*************************************************************** DEFINITIONS */

void setup() {
  Wire.begin();
  Serial.begin(9600); // connects to RaspberryPi control interface (robofriend.py)
  motors.init();
#if MOTOR_CTR_TEST
  motors.test();
#endif
  sensors.init();
  odo.init();
  parser.init();
  wdt_enable(WDTO_8S);
}

void loop() {
  timestamp = micros();
  sensors.readSensorValues();
  parser.processSerialCommands();
  motors.performIntendedMovement();

  loopcounter++;   // used for limiting serial messages etc.

#if DUMP_ENCODER_VALUES
  if (!(loopcounter % 50)) { // this is for printing current speed ! ( should be replaced by speed control algorithm )
    odo.printEncoderValues();
    odo.clearEncoderValues();
  }
#endif

  while (micros() - timestamp < 5000)
    ;
  wdt_reset();
}
