
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
    odo.getStateRightMotor();
    odo.getStateLeftMotor();
    Sensors.updateSensorData();
    parser_processSerialCommands();
    Motors.updateMotors();
 
    if (micros()-timestamp < 5000) 
       delayMicroseconds(5000-(micros()-timestamp));        // main loop runs @ 200Hz 

    loopcounter++;   // used for limiting serial messages etc.
}


