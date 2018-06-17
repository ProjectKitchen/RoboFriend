#include <Arduino.h>
#include "Motor.h"
#include "Sensor.h"

void parser_processSerialCommands() {
    char c = 0;
    if (Serial.available() > 0)
    {
        c = Serial.read();
    
        switch (c)
        {
        case '1': //driving FORWARD
            Motors.drive (255,255,STEPLENGTH);
            break;
        case '2': //driving BACKWARD
            Motors.drive (-255,-255,STEPLENGTH);
            break;
        case '3': //driving RIGHT
            Motors.drive (128,-128,STEPLENGTH_TURN);
            break;
        case '4': //driving LEFT
            Motors.drive (-128,128,STEPLENGTH_TURN);
            break;
        case '5': //driving RIGHT_BACKWARD_LOOP
            Motors.drive (0,-128,-1);
            break;
        case '6': //driving LEFT_BACKWARD_LOOP
            Motors.drive (-128,0,-1);
            break;
        case '7':
            Motors.stop();
            break;
        case '8':
            Sensors.reportSensorValues();
            break;
        case '9':
            // Motors.shake();
            break;
        case 'a': // right forward loop
            Motors.drive (128,0,-1);
            break;
        case 'b': // left forward loop
            Motors.drive (0,128,-1);
            break;
        case 'c':  // forward loop
            Motors.drive (255,255,-1);
            break;
        case 'd': // backward loop
            Motors.drive (-255,-255,-1);
            break;
        case 'e': // right loop
            Motors.drive (128,-128,-1);
            break;
        case 'f': // left loop
            Motors.drive (-128,128,-1);
            break;
        default:
            Serial.println("ERROR: Command '" + (String)c + "' not implemented!");
        }
    }
}
