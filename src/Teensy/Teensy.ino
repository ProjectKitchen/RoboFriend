
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

Sensor Sensors;
Motor  Motors;

void setup()
{
    Serial.begin(9600);   // connects to RaspberryPi control interface (robofriend.py)
    Motors.init();
    Sensors.init();
    legacyPower_init();
    legacyPower_startup();
}


void loop()
{   
    Sensors.updateSensorData();
    parser_processSerialCommands();
    Motors.updateMotors();
}


