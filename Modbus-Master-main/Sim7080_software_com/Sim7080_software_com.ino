#include <SoftwareSerial.h>

SoftwareSerial mySerial(30, 31); // RX, TX

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  
  if (Serial.available())
  mySerial.write(Serial.read());

  if (mySerial.available())
    Serial.write(mySerial.read());

}

