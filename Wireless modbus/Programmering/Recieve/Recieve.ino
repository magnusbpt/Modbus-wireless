
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <EEPROM.h>

#define DTR_E 42
#define PWRKEY 43
#define RED_LED 64
#define GREEN_LED 65
#define BLUE_LED 66
#define Button 67

// #define simSerial Serial3
#define modbusSerial Serial2 
#define loraSerial Serial1

unsigned char* simBuffer = (unsigned char*)malloc(1000 * sizeof(char));
unsigned char serverMessage[500];
unsigned char modBuffer[50];  // Buffer to store modbus slave response
char IMEI[20];                // Array to store IMEI number
char CSQ[3];                  // Til at hente NB signalvaerdi.

static char message[512];
int i = 0;

void setup() {

  // CLKPR = 1 << CLKPCE;  // Clock Prescaler Change Enable
  // CLKPR = 0;            // Change clock division factor to 1.

  Serial.begin(9600);
  // simSerial.begin(9600);
  loraSerial.begin(9600);
  
  //Setup mode of pins on master
  // pinMode(DTR_E, OUTPUT);
  // pinMode(PWRKEY, OUTPUT);
  // pinMode(BLUE_LED, OUTPUT);
  // pinMode(GREEN_LED, OUTPUT);
  // pinMode(RED_LED, OUTPUT);
  // pinMode(Button, INPUT);

  // digitalWrite(BLUE_LED, HIGH);
  // digitalWrite(RED_LED, HIGH);
  // digitalWrite(GREEN_LED, HIGH);
  // digitalWrite(PWRKEY, HIGH);
  // digitalWrite(DTR_E, LOW);

  // millisDelay(3000);

  // simSetup();  //Sim module setup

  // millisDelay(500);

  loraSerial.println(F("AT+MODE=TEST"));
  read();
  // memset(message, 0, sizeof message);

  delay(300);

  loraSerial.println(F("AT+TEST=RFCFG,868,SF12,125,8,8,22,ON,OFF,OFF"));
  read();
  // memset(message, 0, sizeof message);

}

void loop() {
  loraSerial.println(F("AT+TEST=RXLRPKT"));
  read();
  memset(message, 0, sizeof message);

  // digitalWrite(BLUE_LED, LOW);

  while (!loraSerial.available()) {}
  read();

  delay(500);

  // digitalWrite(BLUE_LED, HIGH);


  loraSerial.println("AT+TEST=TXLRPKT, \"BB\"");
  read();
    // memset(message, 0, sizeof message);
}

// void simSetup() {

//   bool start = 0;  // Start bit used in simSetup
//   bool stop = 1;   // Stop bit used in simSetup

//   while (stop) {                   // Stay in loop while stop is set
//     for (int i = 0; i < 4; i++) {  // Check four times for OK response
//       simSerial.println(F("AT"));  // Print AT
//       start = OKcomcheck(300);     // Check for OK
//       if (start) {                 // If start is set, stop while
//         stop = 0;
//       }
//     }
//     if (!start) {  // If start is not set sim module is turned of, therefore turn on
//       simpow();
//     }
//   }
//   stop = 1;  // Set stop for next siminit call

//   clrsimBuffer();

//   // simSerial.println(F("AT+GSN"));  // Sent at command to get IMEI number
//   // responseCheck("OK", 10000);

//   // for (int i = 0; i < 8; i++) {  // Put response into IMEI array
//   //   IMEI[i] = (simBuffer[(i * 2) + 9] - 48) << 4;
//   //   if (i < 7) {
//   //     IMEI[i] = IMEI[i] | (simBuffer[(i * 2) + 10] - 48);
//   //   }
//   // }
//   // clrsimBuffer();  // Clear sim response buffer

//   // simSerial.println(F("AT+CGATT?"));  // Check operator. See operator e.g Telia and if connected to NB or CAT-M
//   // responseCheck("1", 10000);
//   // clrsimBuffer();

//   // simSerial.println(F("AT+COPS?"));  // Check operator. See operator e.g Telia and if connected to NB or CAT-M
//   // if (responseCheck("9", 5000) != 1) {
//   //   responseCheck("7", 5000);
//   // }
//   // clrsimBuffer();

//   // simSerial.println(F("AT+CGNAPN"));
//   // responseCheck("1,\"iot.1nce.net\"", 10000);
//   // clrsimBuffer();

//   // simSerial.println(F("AT+CSQ"));  // Check signal quality
//   // responseCheck("OK", 10000);
//   // for (int i = 0; i < 2; i++) {  // Put response into IMEI array
//   //   CSQ[i] = simBuffer[i + 7];
//   // }
//   // clrsimBuffer();  // Clear sim response buffer

//   // simSerial.println(F("AT+CNCFG=0,1,\"iot.1nce.net\""));  // Check if connected to the correct APN
//   // responseCheck("OK", 10000);
//   // clrsimBuffer();

//   // simSerial.println(F("AT+CNACT=0,1"));  // Activate network
//   // responseCheck("0,ACTIVE", 10000);
//   // clrsimBuffer();

//   // simSerial.println(F("AT+CNACT?"));  // Check if network is active to adress
//   // responseCheck("OK", 10000);
//   // clrsimBuffer();

//   // simSerial.println(F("AT+CAOPEN=0,0,\"UDP\",\"207.154.251.171\",8080"));  // Open UDP connection/socket
//   // responseCheck("OK", 10000);
//   // clrsimBuffer();
// }

// bool OKcomcheck(int dly) {  // Same as comcheck, but only for check "OK" response and with no while

//   bool check = 0;

//   millisDelay(dly);

//   while (simSerial.available()) {
//     int numBytes = simSerial.available();
//     for (int i = 0; i < numBytes; i++) {
//       simBuffer[i] = simSerial.read();
//     }
//   }

//   if (strstr((char*)simBuffer, "OK")) {
//     check = 1;
//   } else {
//     check = 0;
//   }

//   clrsimBuffer();
//   return check;
// }

// void simpow() {  // Used to power on or off sim module

//   digitalWrite(PWRKEY, LOW);
//   millisDelay(1000);
//   digitalWrite(PWRKEY, HIGH);
// }

// int responseCheck(char* c, unsigned int timeout) {  // Check if correct response or ERROR.

//   unsigned long timerStart = 0;
//   unsigned long timerEnd = 0;
//   int check = 0;

//   bool msg = 0;
//   timerStart = millis();

//   while (!msg) {
//     simRead();

//     if (strstr((char*)simBuffer, c)) {
//       check = 1;
//       msg = 1;
//       digitalWrite(GREEN_LED, LOW);
//       millisDelay(125);
//       digitalWrite(GREEN_LED, HIGH);
//       millisDelay(125);
//       digitalWrite(GREEN_LED, LOW);
//       millisDelay(125);
//       digitalWrite(GREEN_LED, HIGH);
//       millisDelay(125);
//     } else if (strstr((char*)simBuffer, "ERROR")) {
//       check = 2;
//       msg = 1;
//       digitalWrite(RED_LED, LOW);
//       millisDelay(125);
//       digitalWrite(RED_LED, HIGH);
//       millisDelay(125);
//       digitalWrite(RED_LED, LOW);
//       millisDelay(125);
//       digitalWrite(RED_LED, HIGH);
//       millisDelay(125);
//     }

//     timerEnd = millis();

//     if (timerEnd - timerStart > timeout) {
//       check = 0;
//       msg = 1;
//       digitalWrite(BLUE_LED, LOW);
//       millisDelay(125);
//       digitalWrite(BLUE_LED, HIGH);
//       millisDelay(125);
//       digitalWrite(BLUE_LED, LOW);
//       millisDelay(125);
//       digitalWrite(BLUE_LED, HIGH);
//       millisDelay(125);
//     }
//   }

//   return check;
// }

// void clrsimBuffer() {  // Clear simBuffer
//   memset(simBuffer, 0, sizeof simBuffer);
// }

// void clrmodBuffer() {  // Clear modBuffer
//   memset(modBuffer, 0, sizeof modBuffer);
// }

void read() {  // Read response after sending AT command

  delay(300);  // Wait for sim module to respons correctly

  while (loraSerial.available()) {  // While data incomming: Read into buffer

  Serial.write(loraSerial.read());
  delay(10);

    int numBytes = loraSerial.available();
    for (int i = 0; i < numBytes; i++) {
      message[i] = loraSerial.read();
    }
  }
  Serial.write((char*)message);  // Write to terminal
  Serial.println();

}

// void simRead() {  // Read response after sending AT command

//   millisDelay(1000);  // Wait for sim module to respons correctly

//   while (simSerial.available()) {  // While data incomming: Read into buffer
//     int numBytes = simSerial.available();
//     for (int i = 0; i < numBytes; i++) {
//       simBuffer[i] = simSerial.read();
//     }
//   }
// }

void millisDelay(int delayTime) {

  unsigned long time_now = millis();

  while (millis() - time_now < delayTime) {
    //wait.
  }
}
