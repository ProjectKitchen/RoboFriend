
/*
 *  Robofriend Teensy++2.0 firmware
 *
 *  implements PWM motor control, sensor management and serial communication with RaspberryPi  
 *  
 *  Version: 9.0 (code modularisation and major rework)
 *  Datum: 06/2018
 */

#include "Motor.h"
#include "Sensor.h"
#include "LegacyPower.h"
#include "Parser.h"
#include "Odometry.h"
#include "GPIO.h"

// #define PRINT_ENCODER_VALUES

Sensor Sensors;
Motor  Motors;
Odometry odo;

long loopcounter=0;
long timestamp;

void setup()
{
    Serial.begin(9600);   // connects to RaspberryPi control interface (robofriend.py)
    Motors.init();
    Sensors.init();
    odo.init();
    
    parser_init();
    legacyPower_init();
    legacyPower_startup();
}


void loop()
{   
    timestamp = micros();
    Sensors.updateSensorData();
    parser_processSerialCommands();
    Motors.updateMotors();
 
    loopcounter++;   // used for limiting serial messages etc.

    #ifdef PRINT_ENCODER_VALUES
    if (!(loopcounter % 50)) {    // this is for printing current speed ! ( should be replaced by speed control algorithm )
      odo.printEncoderValues();
      odo.clearEncoderValues();   
    }
    #endif

    while (micros()-timestamp < 1000);
}
