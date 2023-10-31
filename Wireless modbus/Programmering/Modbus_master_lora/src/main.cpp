#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void read();

#define DTR_E 41
#define PWRKEY 40
#define RED_LED 25
#define GREEN_LED 24
#define BLUE_LED 23
#define Button 22

#define simSerial Serial1 
#define modbusSerial Serial2 
#define loraSerial Serial3

unsigned char* simBuffer = (unsigned char*)malloc(1000 * sizeof(char));
unsigned char serverMessage[500];
unsigned char modBuffer[50];  // Buffer to store modbus slave response
char IMEI[20];                // Array to store IMEI number
char CSQ[3];                  // Til at hente NB signalvaerdi.

char readBuffer[1000];
int i = 0;

void setup() {

  CLKPR = 1 << CLKPCE;  // Clock Prescaler Change Enable
  CLKPR = 0;            // Change clock division factor to 1.

  Serial.begin(9600);
  loraSerial.begin(9600);
  
  pinMode(DTR_E, OUTPUT);
  pinMode(PWRKEY, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(Button, INPUT);

  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(PWRKEY, HIGH);
  digitalWrite(DTR_E, LOW);

  delay(2000);

  loraSerial.println(F("AT+MODE=TEST"));
  read();
  memset(readBuffer, 0, sizeof readBuffer);

  delay(300);

  loraSerial.println(F("AT+TEST=RFCFG,868,SF12,125,8,8,22,ON,OFF,OFF"));
  read();  
  memset(readBuffer, 0, sizeof readBuffer);

  delay(300);

  loraSerial.println(F("AT+TEST=RXLRPKT"));
  read();
  memset(readBuffer, 0, sizeof readBuffer);

  
}

void loop() {

  digitalWrite(BLUE_LED, LOW);

  while (!loraSerial.available()) {}

  read();
  memset(readBuffer, 0, sizeof readBuffer);

  delay(500);

  digitalWrite(BLUE_LED, HIGH);

  delay(500);
  

  // read();

  // if(strstr(readBuffer, "AA")){
  //   memset(readBuffer, 0, sizeof readBuffer);
  //   delay(100);
  //   loraSerial.println("AT+TEST=TXLRPKT, \"BB\"");
  //   read();
  //   memset(readBuffer, 0, sizeof readBuffer);
  // }


}

void read() {    // Read response after sending AT command

  delay(100);      // Wait for sim module to respons correctly
  i = 0;

  while (loraSerial.available()) {           // While data incomming: Read into buffer
      readBuffer[i] = loraSerial.read();
      i++;
  }
  
  Serial.write((char*)readBuffer);    // Write to terminal
  Serial.println();
}
