
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

int LED_PIN =  9;

Sensor Sensors;
Motor  Motors;
Odometry odo;

long loopcounter=0;
long timestamp;

bool led_blink=false;
bool first_loop;

void setup()
{
   pinMode(LED_PIN, OUTPUT) ; 
    digitalWrite(LED_PIN, HIGH);
    
    Serial.begin(115200);   // connects to RaspberryPi control interface (robofriend.py)
    Motors.init();
    Sensors.init();
    odo.init();

    
    parser_init();
    legacyPower_init();
    legacyPower_startup();

   
    first_loop=true;
    
}

/****
 * LED vor update der einzelenen phasen aufdrehen um nach und nach fehler einzugrenzen
 */
void loop()
{   if(first_loop)
    {
      delay(2000);
      first_loop=false;
    }
    timestamp = micros();
    
    /* FEHLER IN DEM BEREICH**********************************/
    Sensors.updateSensorData();//Fehler in dieser Funktion

    
    parser_processSerialCommands();
     
    Motors.updateMotors();
    //odo.updateOdometry();
     
    loopcounter++;   // used for limiting serial messages etc.

 if (!(loopcounter % 100)) { 
        if(led_blink)
      {
        digitalWrite(LED_PIN, HIGH);
        led_blink=false;
      }
      else
      {
        digitalWrite(LED_PIN, LOW);
        led_blink=true;
      }
 }
    
    if (!(loopcounter % 50)) { 
       odo.updateOdometry(); //by higher speed no odom value
      // this is for printing current speed ! ( should be replaced by speed control algorithm )
      //odo.printEncoderValues();
     // odo.clearEncoderValues();   
    // odo.printOdom();
   #ifdef PRINT_ENCODER_VALUES
    odo.printOdom();
    #endif
    

        
   
      
    }
  
    while (micros()-timestamp < 1000);
}
