/*
 * @file      Teensy.ino
 * @version   v9.0
 * @date      01.06.2018
 * @changed   07.03.2019
 * @author    cveigl, mzahedi
 * @brief     Robofriend Teensy++2.0 firmware
 *            Implements PWM motor control, sensor management and serial communication with RaspberryPi
 */

#include "Motor.h"
#include "Sensor.h"
#include "Parser.h"
#include "Odometry.h"
#include "GPIO.h"

// #define PRINT_ENCODER_VALUES

Sensor sensors;
Motor  motors;
Odometry odo;
Parser parser;

long loopcounter=0;
long timestamp;

void setup()
{
    Serial.begin(9600);   // connects to RaspberryPi control interface (robofriend.py)
    motors.init();
    sensors.init();
    odo.init();
    parser.init();
}


void loop()
{   
    timestamp = micros();
    sensors.updateSensorData();
    parser.processSerialCommands();
    motors.updateMotors();
 
    loopcounter++;   // used for limiting serial messages etc.

    #ifdef PRINT_ENCODER_VALUES
    if (!(loopcounter % 50)) {    // this is for printing current speed ! ( should be replaced by speed control algorithm )
      odo.printEncoderValues();
      odo.clearEncoderValues();   
    }
    #endif

    while (micros()-timestamp < 5000);
}
