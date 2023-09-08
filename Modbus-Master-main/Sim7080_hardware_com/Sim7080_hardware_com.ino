/*

  Multiple Serial test

  Receives from the main serial port, sends to the others.

  Receives from serial port 1, sends to the main serial (Serial 0).

  This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc.

  The circuit:

  - any serial device attached to Serial port 1

  - Serial Monitor open on Serial port 0

  created 30 Dec 2008

  modified 20 May 2012

  by Tom Igoe & Jed Roach

  modified 27 Nov 2015

  by Arturo Guadalupi

  This example code is in the public domain.

*/

#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>



void setup() {

  // initialize both serial ports:

  Serial.begin(9600);

  Serial2.begin(9600);

  pinMode(43, OUTPUT);

  digitalWrite(43, HIGH);

}

void loop() {

  // read from port 1, send to port 0:

  if (Serial2.available()) {

    Serial.write(Serial2.read());

  }

  // read from port 0, send to port 1:

  if (Serial.available()) {

    Serial2.write(Serial.read());

  }
}