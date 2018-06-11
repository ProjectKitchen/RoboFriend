
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
 *  8.0: Improvements
 *  
 *
 *  Version: 8.2
 *  Datum: 23.06.2017
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
int STEPLENGTH = 50;
int STEPLENGTH_TURN = 20;

bool SensorMiddleTriggered = false;
bool SensorLeftTriggered = false;
bool SensorRightTriggered = false;

Direction LeftTireDirection = None;
Direction RightTireDirection = None;

/*** global variables ***/
/* Circular Buffer Arrays für IR Sensor Daten */
RunningAverage AbstandLinks(30);
RunningAverage AbstandMitte(30);
RunningAverage AbstandRechts(30);

void setup()
{
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
    digitalWrite(Relais5V, HIGH);
    digitalWrite(Hauptrelais, HIGH);
    digitalWrite(Serienschaltung, HIGH);
    digitalWrite(Relais12V, HIGH);

    /**** INITIALISIERUNG  ****/
    //Opening Serial Port, setting Baudrate to 9600bps
    Serial.begin(9600); //pi

    //Serial.println("5v ein");
    digitalWrite(Relais5V, LOW);
    delay(200);

    //Serial.println("haupt ein");
    digitalWrite(Hauptrelais, LOW);
    delay(500);

    //Serial.println("serie ein");
    digitalWrite(Serienschaltung, LOW);
    delay(200);

    //Serial.println("12V ein");
    digitalWrite(Relais12V, LOW);
    delay(200);
}

void Move(Direction dir, Tire tire)  // sets direction of one wheel
                                     // always call Stop() first to ensure save state of H-bridge!
{
    int motorPin;
    int pwmPin;
    switch (tire)
    {
    case Left:
        LeftTireDirection = dir;
        switch (dir)
        {
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
        RightTireDirection = dir;
        switch (dir)
        {
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

    digitalWrite(motorPin, HIGH);
    digitalWrite(pwmPin, HIGH);
}

void forward()
{
    Stop();
    if (SensorMiddleTriggered || SensorLeftTriggered || SensorRightTriggered)
    {
        return;
    }
    
    Move(Forward, Left);
    Move(Forward, Right);

    for (int i = 0; i < STEPLENGTH; i++)
    {
        delay(10);
        UpdateSensorData();
        if (SensorMiddleTriggered || SensorLeftTriggered || SensorRightTriggered)
        {
            return;
        }
    }

    Stop();
}

void forward_Loop()
{
    Stop();
    if (SensorMiddleTriggered || SensorLeftTriggered || SensorRightTriggered)
    {
        return;
    }
    
    Move(Forward, Left);
    Move(Forward, Right);
}

void backward()
{
    Stop();
    Move(Backward, Left);
    Move(Backward, Right);

    delay(STEPLENGTH*10);

    Stop();
}

void backward_Loop()
{
    Stop();
    Move(Backward, Left);
    Move(Backward, Right);
}

void left()
{    
    Stop();
    Move(Backward, Left);
    Move(Forward, Right);

    for (int i = 0; i < STEPLENGTH_TURN; i++)
    {
        delay(10);
    }

    Stop();
}

void left_Loop()
{    
    Stop();
    Move(Backward, Left);
    Move(Forward, Right);
}

void right()
{    
    Stop();
    Move(Forward, Left);
    Move(Backward, Right);

    for (int i = 0; i < STEPLENGTH_TURN; i++)
    {
        delay(10);
    }

    Stop();
}

void right_Loop()
{
    Stop();
    Move(Forward, Left);
    Move(Backward, Right);
}

void right_forward_Loop()
{
    Stop();
    if (SensorMiddleTriggered || SensorRightTriggered)
    {
        return;
    }
    
    Move(Forward, Left);
}

void left_forward_Loop()
{
    Stop();
    if (SensorMiddleTriggered || SensorLeftTriggered)
    {
        return;
    }
    
    Move(Forward, Right);
}

void right_backward_Loop()
{
    Stop();
    Move(Backward, Left);
}

void left_backward_Loop()
{
    Stop();
    Move(Backward, Right);
}

void shake()
{
    Stop();
    Move(Backward, Left);
    Move(Forward, Right);

    delay(250);

    Stop();
    Move(Forward, Left);
    Move(Backward, Right);

    delay(250);

    Stop();
}

void UpdateSensorData()
{
    if (LeftTireDirection == Backward || RightTireDirection == Backward)
    {
        return;
    }
    
    for (int i = 0; i < 30; i++)
    {
        AbstandLinks.addValue(analogRead(38));
        AbstandMitte.addValue(analogRead(39));
        AbstandRechts.addValue(analogRead(40));
    }

    SensorMiddleTriggered = AbstandMitte.getFastAverage() >= SensorMiddleThreshold;
    SensorLeftTriggered = AbstandLinks.getFastAverage() >= SensorLeftThreshold;
    SensorRightTriggered = AbstandRechts.getFastAverage() >= SensorRightThreshold;

    if (LeftTireDirection == Forward || RightTireDirection == Forward)
    {
        if (SensorMiddleTriggered)
        {
            Stop();
        }
        else if (RightTireDirection == Forward && SensorLeftTriggered)
        {
            Stop();
        }
        else if (LeftTireDirection == Forward && SensorRightTriggered)
        {
            Stop();
        }
    }
}

void Stop()
{
    LeftTireDirection = None;
    RightTireDirection = None;

    //PWMs aus
    digitalWrite(LeftPWM, LOW);
    digitalWrite(RightPWM, LOW);

    //Richtungen aus
    digitalWrite(LeftForward, LOW);
    digitalWrite(RightBackward, LOW);
    digitalWrite(LeftBackward, LOW);
    digitalWrite(RightForward, LOW);
    delay (100);  // to ensure save state of H-brigde
}

void loop()
{
    //Battery status information
    int Battery = analogRead(41);
    //Serial.println(Battery);
   
    UpdateSensorData();
    char c = 0;
    if (Serial.available() > 0)
    {
        c = Serial.read();
    
        switch (c)
        {
        case '1': //driving FORWARD
            forward();
            break;
        case '2': //driving BACKWARD
            backward();
            break;
        case '3': //driving RIGHT
            right();
            break;
        case '4': //driving LEFT
            left();
            break;
        case '5': //driving RIGHT_BACKWARD_LOOP
            right_backward_Loop();
            break;
        case '6': //driving LEFT_BACKWARD_LOOP
            left_backward_Loop();
            break;
        case '7':
            Stop();
            break;
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
        /*
        case 'n':
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
        default:
            Serial.println("ERROR: Command '" + (String)c + "' not implemented!");
        }
    }
}
