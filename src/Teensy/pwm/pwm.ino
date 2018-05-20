#include "support.h"

/* RGB Analog Example, Teensyduino Tutorial #2
   http://www.pjrc.com/teensy/tutorial2.html

   This example code is in the public domain.
*/

const u8 redPin =  0x0C; // 12
const u8 greenPin =  0x0F; // 15
const u8 bluePin =  0x0E; // 14

void setup() {                
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop()                     
{
  analogWrite(redPin, 30);
  delay(500);
  analogWrite(greenPin, 200);
  delay(500);
  analogWrite(bluePin, 40);
  delay(500);
  analogWrite(redPin, 150);
  delay(500);
  analogWrite(greenPin, 0);
  delay(500);
  analogWrite(bluePin, 250);
  delay(500);
}

