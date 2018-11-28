#include <Arduino.h>
#include "Motor.h"
#include "Sensor.h"
#include "SerialCommand.h"

SerialCommand SCmd;   // The demo SerialCommand object

void drive()
{
  int left=0,right=0,duration=-1;  
  char *arg; 

  arg = SCmd.next(); 
  if (arg != NULL) {
    left=atoi(arg);    // Converts a char string to an integer
  } 
  arg = SCmd.next(); 
  if (arg != NULL)  {
    right=atoi(arg);
  } 
  arg = SCmd.next(); 
  if (arg != NULL)  {
    duration=atoi(arg)/10;
  } 
  Motors.drive(left,right,duration);
}

void report_sensorValues()
{
  Sensors.reportSensorValues();
}

void set_sensorThresholds()
{
  int left=DEFAULT_IR_LEFT_THRESHOLD;
  int middle=DEFAULT_IR_MIDDLE_THRESHOLD;
  int right=DEFAULT_IR_RIGHT_THRESHOLD;  
  char *arg; 

  arg = SCmd.next(); 
  if (arg != NULL) {
    left=atoi(arg);
  } 
  arg = SCmd.next(); 
  if (arg != NULL) {
    middle=atoi(arg);
  } 
  arg = SCmd.next(); 
  if (arg != NULL) {
    right=atoi(arg);
  } 
  Sensors.setSensorThresholds(left,middle,right);
}

void unsupported_command() {
  Serial.println("Unsupported command");
}

void parser_init()
{
  SCmd.addCommand("D",drive);  
  SCmd.addCommand("R",report_sensorValues);  
  SCmd.addCommand("S",set_sensorThresholds);  
  SCmd.addDefaultHandler(unsupported_command);
}

void parser_processSerialCommands() {
  SCmd.readSerial(); 
}
