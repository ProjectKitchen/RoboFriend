#include <Arduino.h>
#include "LegacyPower.h"


void legacyPower_init() {
    // set pins to output
    pinMode(Hauptrelais, OUTPUT);
    pinMode(Serienschaltung, OUTPUT);

    pinMode(Relais5V, OUTPUT);
    pinMode(Relais12V, OUTPUT);
}

void legacyPower_startup() {

    // switch all relais off !  (high = off!!)
    digitalWrite(Relais5V, HIGH);
    digitalWrite(Hauptrelais, HIGH);
    digitalWrite(Serienschaltung, HIGH);
    digitalWrite(Relais12V, HIGH);

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


void legacyPower_shutdown() {
    //Serial.println("12V aus");
    digitalWrite (Relais12V, HIGH);
    delay(200);

    //Serial.println("serie aus");
    digitalWrite (Serienschaltung, HIGH);
    delay(200);

    //Serial.println("haupt aus");
    digitalWrite (Hauptrelais, HIGH);
    delay(200);

    //Serial.println("5V aus");
    digitalWrite (Relais5V, HIGH);
    delay(5000);
}
