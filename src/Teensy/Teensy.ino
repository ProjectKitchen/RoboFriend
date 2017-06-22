
/************************
 *  Teensy
 *  based on robot007.bas for C-Control
 *
 *  "Sofortige und permanente Versorgung des Mainboards inkl 12V.Relais.
 *  Motorsteuerung mit seriellen Kommandos vom Mainboard oder einem Terminal.
 *  Programmende nur durch Abschalten"
 *
 *  Ser. Kommunikation via USB zu RasPi, Steuerung der Motoren.
 *  abstandssensoren...; berechnung vom Durchschnitt, bei zu geringem Abstand
 *  ist fahren nicht möglich
 *
 *  5.1: minor adjustments regarding comments
 *  5.2: adding println commands
 *  6.0: adding port reading 41 for battery information
 *  6.1: improvement of driving
 *  7.0: adding additionially drive commandos
 *  7.1: adjusting new drive commands and expanded switch case
 *
 *  Version: 6.1
 *  Datum: 02.04.2017
 ************************/

/**** included Libraries ****/
#include "RunningAverage.h" // Library used for IR Sensor Data
#include "Definitions.h"

 /**********************
  *  IR Sensoren Positionen
  *  AbstandLinks = 38;
  *  AbstandMitte = 39;
  *  AbstandRechts = 40; --> entspricht dem Port der am nähersten zum Teensy is
************************/

/******** PINBELEGUNG ********/
const int Hauptrelais = 20;
const int Serienschaltung = 21;
const int Relais5V = 23;
const int Relais12V = 22;
/* Motorsteuerung Rechts */
const int RightBackward = 10;
const int RightForward = 11;
const int RightPWM = 14;
/* Motorsteuerung Links */
const int LeftForward = 12;
const int LeftBackward = 13;
const int LeftPWM = 15;

int SensorMiddleThreshold = 150;
int SensorLeftThreshold = 90;
int SensorRightThreshold = 80;
int STEPLENGTH = 90;


int FW = 0;
int BW = 0;
int DriveRight = 0;
int DriveLeft = 0;

/*** global variables ***/
/* Circular Buffer Arrays für IR Sensor Daten */
RunningAverage AbstandLinks(30);
RunningAverage AbstandMitte(30);
RunningAverage AbstandRechts(30);

void setup() {
  // put your setup code here, to run once:
  /**** PINMODE DECLARATION ****/
  pinMode(Hauptrelais, OUTPUT);
  pinMode(Serienschaltung, OUTPUT);

  pinMode(Relais5V, OUTPUT);
  pinMode(Relais12V, OUTPUT);

  pinMode(RightBackward, OUTPUT);
  pinMode(RightForward, OUTPUT);
  pinMode(RightPWM, OUTPUT);

  pinMode(LeftForward, OUTPUT);
  pinMode(LeftBackward, OUTPUT);
  pinMode(LeftPWM, OUTPUT);

   // switch all relais off !  (high = off!!)
  digitalWrite (Relais5V, HIGH);
  digitalWrite (Hauptrelais, HIGH);
  digitalWrite (Serienschaltung, HIGH);
  digitalWrite (Relais12V, HIGH);

  /**** INITIALISIERUNG  ****/
  //Opening Serial Port, setting Baudrate to 9600bps
  Serial.begin(9600); //pi

  //Serial.println("5v ein");
  digitalWrite (Relais5V, LOW);
  delay(200);

  //Serial.println("haupt ein");
  digitalWrite (Hauptrelais, LOW);
  delay(500);

  //Serial.println("serie ein");
  digitalWrite (Serienschaltung, LOW);
  delay(200);

  //Serial.println("12V ein");
  digitalWrite (Relais12V, LOW);
  delay(200);
}

void Move(Direction dir, Tire tire, bool drive) {
  int motorPin;
  int pwmPin;
  switch(tire){
    case Left:
    switch (dir){
      case Forward:
      motorPin = LeftForward;
      break;
      case Backward:
      motorPin = LeftBackward;
      break;
      default:
      return;
    }

    pwmPin = LeftPWM;
    break;
    case Right:
    switch (dir){
      case Forward:
      motorPin = RightForward;
      break;
      case Backward:
      motorPin = RightBackward;
      break;
      default:
      return;
    }

    pwmPin = RightPWM;
    break;
    default:
    return;
  }

  if (drive) {
    // Move
    digitalWrite (motorPin, HIGH);
    digitalWrite (pwmPin, HIGH);
  } else {
    // Stop
    digitalWrite (pwmPin, LOW);
    digitalWrite (motorPin, LOW);
  }
}

void forward() {
  int i = 0;
  stopping();

  Move(Forward, Left, true);
  Move(Forward, Right, true);

  for ( i=0; i< STEPLENGTH; i++){
    UpdateSensorData();
    delay(10);
  }

  Move(Forward, Left, false);
  Move(Forward, Right, false);
}

void forward_Loop() {
  stopping();
  FW = 1;

  Move(Forward, Left, true);
  Move(Forward, Right, true);
}

void backward(){
  stopping();

  Move(Backward, Left, true);
  Move(Backward, Right, true);

  delay(2000);

  Move(Backward, Left, false);
  Move(Backward, Right, false);
}

void backward_Loop(){
  stopping();
  BW = 1;

  Move(Backward, Left, true);
  Move(Backward, Right, true);
}

void left(){
  int i = 0;
  stopping();

  Move(Backward, Left, true);
  Move(Forward, Right, true);

  for ( i=0; i< STEPLENGTH; i++){
    UpdateSensorData();
    delay(10);
  }

  Move(Backward, Left, false);
  Move(Forward, Right, false);
}

void left_Loop(){
  stopping();

  Move(Backward, Left, true);
  Move(Forward, Right, true);
}

void right(){
  int i = 0;
  stopping();

  Move(Forward, Left, true);
  Move(Backward, Right, true);

  for ( i=0; i< STEPLENGTH; i++){
    UpdateSensorData();
    delay(10);
  }

  Move(Forward, Left, false);
  Move(Backward, Right, false);
}

void right_Loop(){
  stopping();

  Move(Forward, Left, true);
  Move(Backward, Right, true);
}

void right_forward_Loop(){
  stopping();
  DriveRight = 1;
  FW = 1;

  Move(Forward, Left, true);
}

void left_forward_Loop(){
  stopping();
  DriveLeft = 1;
  FW = 1;

  Move(Forward, Right, true);
}

void right_backward_Loop(){
  stopping();
  BW = 1;
  DriveRight = 1;

  Move(Backward, Left, true);
}

void left_backward_Loop(){
  stopping();
  DriveLeft = 1;
  BW = 1;

  Move(Backward, Right, true);
}

void shake(){
  Move(Backward, Left, true);
  Move(Forward, Right, true);

  delay(250);

  Move(Backward, Left, false);
  Move(Forward, Right, false);

  Move(Forward, Left, true);
  Move(Backward, Right, true);

  delay(250);

  Move(Forward, Left, false);
  Move(Backward, Right, false);
}

void UpdateSensorData(){
  for (int i = 0; i < 30; i++) {
    AbstandLinks.addValue(analogRead(38));
    AbstandMitte.addValue(analogRead(39));
    AbstandRechts.addValue(analogRead(40));
  }

  if (FW==1) {
    if (AbstandMitte.getFastAverage() >= SensorMiddleThreshold) {
      stopping();
      return;
    }

    if (DriveLeft == 1 && (AbstandLinks.getFastAverage() >= SensorLeftThreshold)) {
      stopping();
    } else if (DriveRight == 1 && (AbstandRechts.getFastAverage() >= SensorRightThreshold)) {
      stopping();
    } else if ((AbstandLinks.getFastAverage() >= SensorLeftThreshold) || (AbstandRechts.getFastAverage() >= SensorRightThreshold)) {
      stopping();
    }
  }
}

void stopping() {
  FW = 0;
  DriveLeft = 0;
  BW = 0;
  DriveRight = 0;

  //PWMs aus
  digitalWrite (LeftPWM, LOW);
  digitalWrite (RightPWM, LOW);

  //Richtungen aus
  digitalWrite (LeftForward, LOW);
  digitalWrite (RightBackward, LOW);
  digitalWrite (LeftBackward, LOW);
  digitalWrite (RightForward, LOW);
}

void loop() {
  UpdateSensorData();

  //Battery status information
 int Battery = analogRead(41);
 //Serial.println(Battery);

  if (Serial.available()){
     char c = Serial.read();
    switch (c) {
      case '1':  //driving FORWARD
        forward();
        break;
      case '2':  //driving BACKWARD
        backward();
        break;
      case '3':  //driving RIGHT
        right();
        break;
      case '4':  //driving LEFT
        left();
        break;
      case '5':  //driving RIGHT_BACKWARD_LOOP
        right_backward_Loop();
        break;
      case '6':  //driving LEFT_BACKWARD_LOOP
        left_backward_Loop();
        break;
      case '7':
        stopping();
        break;
        /*
        case 'nuke':
        Serial.println("12V aus");
        digitalWrite (Relais12V, HIGH);
        delay(200);

        Serial.println("serie aus");
        digitalWrite (Serienschaltung, HIGH);
        delay(200);

        Serial.println("haupt aus");
        digitalWrite (Hauptrelais, HIGH);
        delay(200);

        Serial.println("5V aus");
        digitalWrite (Relais5V, HIGH);
        delay(5000);
        break;*/
      case '8':
        Battery = analogRead(41);
        Serial.println(Battery);
        break;
      case '9':
        shake();
        break;
      case 'a':
        //driving RIGHT_FORWARD loop
        right_forward_Loop();
        break;
      case 'b':
        //driving LEFT_FORWARD loop
        left_forward_Loop();
        break;
      case 'c':
        //driving FORWARD loop
        forward_Loop();
        break;
      case 'd':
        //driving BACKWARD loop
        backward_Loop();
        break;
      case 'e':
        //driving RIGHT loop
        right_Loop();
        break;
      case 'f': // driving LEFT loop
        left_Loop();
        break;
      default:
        Serial.println("ERROR: Command '" + (String)c + "' not implemented!");
    }
  }
}
