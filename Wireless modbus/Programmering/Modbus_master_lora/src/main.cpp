#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DTR_E 41
#define PWRKEY 40
#define RED_LED 25
#define GREEN_LED 24
#define BLUE_LED 23
#define Button 22

/********Define swith case numbers**********/
#define slaveread 0
#define serversend 1
#define slavewrite 2
#define poweroff 3

#define simSerial Serial1
#define modbusSerial Serial2
#define loraSerial Serial3

unsigned char *simBuffer = (unsigned char *)malloc(1000 * sizeof(char));
char loraBuffer[200];
char messageBuffer[500];
unsigned char modBuffer[50]; // Buffer to store modbus slave response
char IMEI[20];               // Array to store IMEI number
char CSQ[3];                 // Til at hente NB signalvaerdi.
char IDarray[60];

bool slaveSetupCheck = 0;
byte transactionID = 0;
int state = 0;
int ADCValue = 0;
float voltage = 0;
int msgLength = 0;
unsigned int loraLenght = 0;

/***************Define functions***************/
void loraRead();
void millisDelay(unsigned int delayTime);
void simSetup();
void simpow();
bool OKcomcheck(int dly);
int responseCheck(char *c, unsigned int timeout);
void clrsimBuffer();
void simRead();
unsigned short CRC16_modbus(char *buf, int len);
void serverRead();
void sendToServer();
void loraSlaveRead();
bool powerStatus();
uint16_t CRC16 (char *nData, uint16_t wLength);

void setup()
{

  CLKPR = 1 << CLKPCE; // Clock Prescaler Change Enable
  CLKPR = 0;           // Change clock division factor to 1.

  Serial.begin(9600);
  simSerial.begin(9600);
  loraSerial.begin(9600);

  pinMode(DTR_E, OUTPUT);
  pinMode(PWRKEY, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(Button, INPUT);

  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(PWRKEY, HIGH);

  millisDelay(3000);

  simSetup(); // Sim module setup

  /**********Send model and firmware***************/

  char modelAndFirmware[20]; // Create message array
  short model = 0x0001;      // Model nr.
  short firmware = 0x0001;   // Firmware nr.

  for (unsigned int i = 0; i < strlen(IMEI); i++)
  { // Put IMEI into message
    modelAndFirmware[i] = IMEI[i];
  }

  modelAndFirmware[8] = transactionID++; // Put transactionID into message

  modelAndFirmware[9] = 0x02; // Put command 02 into message

  modelAndFirmware[10] = highByte(model); // Put model nr. into message
  modelAndFirmware[11] = lowByte(model);

  modelAndFirmware[12] = highByte(firmware); // Put firmware nr. into massage
  modelAndFirmware[13] = lowByte(firmware);

  modelAndFirmware[14] = highByte(CRC16_modbus(modelAndFirmware, 14)); // Put CRC into message
  modelAndFirmware[15] = lowByte(CRC16_modbus(modelAndFirmware, 14));

  simSerial.println(F("AT+CASEND=0,16")); // Write to send message 16 bytes long
  millisDelay(200);
  clrsimBuffer();

  simSerial.write(modelAndFirmware, sizeof(modelAndFirmware)); // Send model and firmware
  responseCheck("OK", 2000);
  clrsimBuffer();

  simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server
  responseCheck("49", 2000);
  clrsimBuffer();

  /**********Ask for slave setup on startup********/
  // char msg[15]; // Create message array

  // for (int i = 0; i < strlen(IMEI); i++)
  // { // Add IMEI to message
  //   msg[i] = IMEI[i];
  // }

  // msg[8] = transactionID++; // Add transactionID

  // msg[9] = 0x03; // Add command 03

  // msg[10] = highByte(CRC16_modbus(msg, 10)); // Add CRC
  // msg[11] = lowByte(CRC16_modbus(msg, 10));

  // simSerial.println(F("AT+CASEND=0,12")); // Write to send message 12 bytes long
  // millisDelay(500);
  // clrsimBuffer();

  // simSerial.write(msg, sizeof(msg)); // Send message
  // responseCheck("OK", 2000);
  // clrsimBuffer();

  // simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server

  // serverRead(); // Read message from server into simBuffer

  // for (int i = 0; i < strlen(simBuffer); i++) {
  //   setupMessage[i] = simBuffer[i];
  // }
}

void loop()
{

  switch (state)
  {
  case slaveread:

    loraSlaveRead();
    break;

  case serversend:
    sendToServer();

    simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server

    serverRead(); // Read server message
    clrsimBuffer();

    simSerial.println(F("AT+CACLOSE=0")); // Check if connected to the correct APN
    responseCheck("OK", 300);
    clrsimBuffer();

    simSerial.println(F("AT+CAOPEN=0,0,\"UDP\",\"164.92.164.168\",8080")); // Open UDP connection/socket
    responseCheck("OK", 2000);
    clrsimBuffer();

    state = slaveread;

    break;

  case slavewrite:
    break;

  case poweroff:

    char masterData[20]; // Create message array

    for (unsigned int i = 0; i < strlen(IMEI); i++)
    { // Add IMEI
      masterData[i] = IMEI[i];
    }
    masterData[8] = transactionID++;

    masterData[9] = 0x01;

    masterData[10] = highByte(0x0001);
    masterData[11] = lowByte(0x0001);

    masterData[12] = 0x00;

    masterData[13] = highByte(0x0001);
    masterData[14] = lowByte(0x0001);

    masterData[15] = highByte(0x0001);
    masterData[16] = lowByte(0x0001);

    masterData[17] = highByte(CRC16_modbus(masterData, 17));
    masterData[18] = lowByte(CRC16_modbus(masterData, 17));

    simSerial.println(F("AT+CASEND=0,19")); // Write to send message 19 bytes long
    millisDelay(300);
    clrsimBuffer();

    simSerial.write(masterData, 19); // Send message
    responseCheck("OK", 2000);

    simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server
    responseCheck("00", 2000);

    state = slaveread;
    break;
  }
}

void loraSlaveRead()
{
  loraSerial.println(F("AT+MODE=TEST"));
  loraRead();
  memset(loraBuffer, 0, sizeof loraBuffer);

  millisDelay(100);

  loraSerial.println(F("AT+TEST=RFCFG,868,SF12,125,8,8,22,ON,OFF,OFF"));
  loraRead();
  memset(loraBuffer, 0, sizeof loraBuffer);

  millisDelay(100);

  loraSerial.println(F("AT+TEST=RXLRPKT"));
  loraRead();
  memset(loraBuffer, 0, sizeof loraBuffer);

  digitalWrite(BLUE_LED, LOW);

  while (!loraSerial.available())
  {
  }
  delay(100);
  digitalWrite(BLUE_LED, HIGH);

  loraRead();

  unsigned int bufferLen = strlen(loraBuffer) / 2;

  for (unsigned int i = 0; i < bufferLen; i++)
  {
    if (loraBuffer[i * 2] < 0x3A)
    {
      messageBuffer[i] = (loraBuffer[i * 2] - 48) << 4;
    }
    else
    {
      messageBuffer[i] = (loraBuffer[i * 2] - 55) << 4;
    }

    if (loraBuffer[(i * 2) + 1] < 0x3A)
    {
      messageBuffer[i] = messageBuffer[i] | (loraBuffer[(i * 2) + 1] - 48);
    }
    else
    {
      messageBuffer[i] = messageBuffer[i] | (loraBuffer[(i * 2) + 1] - 55);
    }
  }

  unsigned short CRCcheck = (messageBuffer[bufferLen - 2] << 8) | messageBuffer[bufferLen - 1];

  messageBuffer[bufferLen - 2] = lowByte(CRCcheck);
  messageBuffer[bufferLen - 1] = highByte(CRCcheck);

  unsigned short CRC = CRC16_modbus(messageBuffer, strlen(messageBuffer) - 2);
 
  messageBuffer[bufferLen] = highByte(CRC);
  messageBuffer[bufferLen + 1] = lowByte(~CRC);
 

  simSerial.println("AT+CASEND=0,9"); // Need ln when writing to sim module

  millisDelay(300);

  simSerial.write(messageBuffer, 9); // Send message to server

  millisDelay(300);

  simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server
  serverRead();                             // Read server message
  clrsimBuffer();

  memset(messageBuffer, 0, sizeof messageBuffer);
  memset(loraBuffer, 0, sizeof loraBuffer);

  

  // if (CRC == CRCcheck)
  // {
  //   loraSerial.println(F("AT+MODE=TEST"));
  //   loraRead();
  //   memset(loraBuffer, 0, sizeof loraBuffer);

  //   millisDelay(1000);

  //   char loraTX[50] = "AT+TEST=TXLRPKT,\"";

  //   char returnID[1];
  //   returnID[0] = messageBuffer[0];

  //   strcat(loraTX, returnID);
  //   strcat(loraTX, "\"\r\n");

  //   loraSerial.write(loraTX, strlen(loraTX));
  //   millisDelay(800);
  //   loraRead();
  //   memset(loraBuffer, 0, sizeof loraBuffer);

  //   state = serversend; // Change state to send message to server
  // }
}

void sendToServer()
{
  char serverMessage[500];
  int msgPos = 12;

  int len = strlen(messageBuffer);

  if (len > 4)
  {

    for (unsigned int i = 0; i < strlen(IMEI); i++)
    { // Put IMEI into message
      serverMessage[i] = IMEI[i];
    }

    serverMessage[8] = transactionID++; // Change transactionID for new message

    serverMessage[9] = 0x01; // Add command 04

    serverMessage[10] = 0x00;
    serverMessage[11] = (len - 3) / 4; // Add length of message

    for (unsigned int i = 0; i < len - 2; i++)
    { // Add message
      serverMessage[msgPos++] = messageBuffer[i];
    }

    unsigned short CRC1 = CRC16_modbus(serverMessage, msgPos);

    serverMessage[msgPos] = highByte(CRC1); // Add CRC
    msgPos++;
    serverMessage[msgPos] = lowByte(CRC1);
  }
  else
  {
    for (unsigned int i = 0; i < strlen(IMEI); i++)
    { // Put IMEI into message
      serverMessage[i] = IMEI[i];
    }

    serverMessage[8] = transactionID++; // Change transactionID for new message

    msgPos = 9;

    serverMessage[msgPos++] = 0x00; // Add command 00 for heartbeat

    serverMessage[msgPos++] = messageBuffer[0];

    unsigned short CRC2 = CRC16_modbus(serverMessage, msgPos);

    // Find CRC from message
    serverMessage[msgPos] = highByte(CRC2); // Put CRC into message
    msgPos++;
    serverMessage[msgPos] = lowByte(CRC2);
  }

  char sendLength[20] = "AT+CASEND=0,"; // To hold at command that sends size of data to be send
  char msgLength[20];                   // Hold size of message

  sprintf(msgLength, "%u", msgPos + 1); // Add message size to end of AT command
  strcat(sendLength, msgLength);

  simSerial.write(sendLength, sizeof(sendLength)); // Send AT command
  simSerial.println();                             // Need ln when writing to sim module

  millisDelay(300);

  simSerial.write(serverMessage, msgPos + 1); // Send message to server

  millisDelay(300);

  memset(messageBuffer, 0, sizeof messageBuffer);
}

void simSetup()
{

  bool start = 0; // Start bit used in simSetup
  bool stop = 1;  // Stop bit used in simSetup

  while (stop)
  { // Stay in loop while stop is set
    for (int i = 0; i < 5; i++)
    {                             // Check four times for OK response
      simSerial.println(F("AT")); // Print AT
      start = OKcomcheck(300);    // Check for OK
      if (start)
      { // If start is set, stop while
        stop = 0;
      }
    }
    if (!start)
    { // If start is not set sim module is turned of, therefore turn on
      simpow();
    }
  }
  start = 0; // Start bit used in simSetup
  stop = 1;  // Set stop for next siminit call

  clrsimBuffer(); // Clear sim response buffer

  clrsimBuffer(); // Clear sim response buffer

  simSerial.println(F("AT+CPIN?")); // Check SIM card status
  responseCheck("READY", 2000);
  clrsimBuffer();

  simSerial.println(F("AT+GSN")); // Sent at command to get IMEI number
  responseCheck("OK", 2000);

  for (int i = 0; i < 7; i++)
  { // Put response into IMEI array
    if (i < 1)
    {
      IMEI[i] = (simBuffer[9] - 48);
    }
    IMEI[i + 1] = ((simBuffer[(i * 2) + 10] - 48) << 4) | (simBuffer[(i * 2) + 11] - 48);
  }
  clrsimBuffer(); // Clear sim response buffer

  simSerial.println(F("AT+CGATT?")); // Check PS Service
  responseCheck("1", 2000);
  clrsimBuffer();

  simSerial.println(F("AT+CNCFG=0,1,\"iot.1nce.net\"")); // Check if connected to the correct APN
  if (responseCheck("OK", 2000) != 1)
  {                                    // If not connected
    simSerial.println(F("AT+CFUN=0")); // Set phone functionality to minimal
    responseCheck("OK", 2000);
    clrsimBuffer();

    simSerial.println(F("AT+CGDCONT=1,\"IP\",\"iot.1nce.net\"")); // Define PDP contect: PDP contect identifier, Set to IP, APN
    responseCheck("OK", 2000);
    clrsimBuffer();

    simSerial.println(F("AT+CFUN=1")); // Set phone funtionality to full
    responseCheck("OK", 2000);
    clrsimBuffer();

    simSerial.println(F("AT+COPS=1,2,\"23820\"")); // Set phone funtionality to full
    responseCheck("OK", 2000);
    clrsimBuffer();
  }
  clrsimBuffer();

  simSerial.println(F("AT+COPS?")); // Check operator. See operator e.g Telia and if connected to NB or CAT-M
  if (responseCheck("9", 2000) != 1)
  {
    responseCheck("7", 2000);
  }
  clrsimBuffer();

  simSerial.println(F("AT+CGNAPN")); // Check if connected to the correct APN
  responseCheck("\"iot.1nce.net\"", 2000);
  clrsimBuffer();

  simSerial.println(F("AT+CSQ")); // Check signal quality
  responseCheck("OK", 2000);
  for (int i = 0; i < 2; i++)
  { // Put response into IMEI array
    CSQ[i] = simBuffer[i + 7];
  }
  clrsimBuffer(); // Clear sim response buffer

  simSerial.println(F("AT+CNACT=0,1")); // Activate network
  responseCheck("0,ACTIVE", 2000);
  clrsimBuffer();

  simSerial.println(F("AT+CNACT?")); // Check if network is active to adress
  responseCheck("OK", 2000);
  clrsimBuffer();

  simSerial.println(F("AT+CAOPEN=0,0,\"UDP\",\"164.92.164.168\",8080")); // Open UDP connection/socket
  responseCheck("0,0", 2000);
  clrsimBuffer();
}

bool powerStatus()
{
  ADCValue = analogRead(A0);
  voltage = ADCValue * (5.0 / 1023.0);

  if (voltage > 2)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void loraRead()
{ // Read response after sending AT command

  char tempBuffer[500];

  millisDelay(200); // Wait for sim module to respons correctly
  int i = 0;
  int j = 0;

  while (loraSerial.available())
  { // While data incomming: Read into buffer
    tempBuffer[i] = loraSerial.read();
    i++;
    if (strstr(tempBuffer, "RX ") > 0)
    {
      tempBuffer[i] = loraSerial.read();
      while (loraSerial.available())
      {
        loraBuffer[j] = loraSerial.read();
        if (strstr(loraBuffer, "\""))
        {
          loraBuffer[j] = '\0';
          break;
        }
        j++;
      }
      memset(tempBuffer, 0, sizeof tempBuffer);
    }
  }
}

void millisDelay(unsigned int delayTime)
{

  unsigned long time_now = millis();

  while (millis() - time_now < delayTime)
  {
    // wait.
  }
}

bool OKcomcheck(int dly)
{ // Same as comcheck, but only for check "OK" response and with no while

  bool check = 0;

  millisDelay(dly);

  if (simSerial.available())
  {
    int numBytes = simSerial.available();
    for (int i = 0; i < numBytes; i++)
    {
      simBuffer[i] = simSerial.read();
    }
  }

  if (strstr((char *)simBuffer, "OK"))
  {
    check = 1;
  }
  else
  {
    check = 0;
  }

  clrsimBuffer();
  return check;
}

void simpow()
{ // Used to power on or off sim module

  digitalWrite(PWRKEY, LOW);
  millisDelay(1500);
  digitalWrite(PWRKEY, HIGH);
}

void clrsimBuffer()
{ // Clear simBuffer
  memset(simBuffer, 0, sizeof simBuffer);
}

int responseCheck(char *c, unsigned int timeout)
{ // Check if correct response or ERROR.

  unsigned long timerStart = 0;
  unsigned long timerEnd = 0;
  int check = 0;

  bool msg = 0;
  timerStart = millis();

  while (!msg)
  {
    simRead();

    if (strstr((char *)simBuffer, c) > 0)
    {
      check = 1;
      msg = 1;
      digitalWrite(GREEN_LED, LOW);
      millisDelay(125);
      digitalWrite(GREEN_LED, HIGH);
      millisDelay(125);
      digitalWrite(GREEN_LED, LOW);
      millisDelay(125);
      digitalWrite(GREEN_LED, HIGH);
      millisDelay(125);
    }
    else if (strstr((char *)simBuffer, "ERROR") > 0)
    {
      check = 2;
      msg = 1;
      digitalWrite(RED_LED, LOW);
      millisDelay(125);
      digitalWrite(RED_LED, HIGH);
      millisDelay(125);
      digitalWrite(RED_LED, LOW);
      millisDelay(125);
      digitalWrite(RED_LED, HIGH);
      millisDelay(125);
    }

    timerEnd = millis();

    if (timerEnd - timerStart > timeout)
    {
      check = 0;
      msg = 1;
      digitalWrite(BLUE_LED, LOW);
      millisDelay(125);
      digitalWrite(BLUE_LED, HIGH);
      millisDelay(125);
      digitalWrite(BLUE_LED, LOW);
      millisDelay(125);
      digitalWrite(BLUE_LED, HIGH);
      millisDelay(125);
    }
    clrsimBuffer(); // Clear simBuffer
  }

  return check;
}

void simRead()
{ // Read response after sending AT command

  millisDelay(300); // Wait for sim module to respons correctly

  while (simSerial.available())
  { // While data incomming: Read into buffer
    int numBytes = simSerial.available();
    for (int i = 0; i < numBytes; i++)
    {
      simBuffer[i] = simSerial.read();
    }
  }
}

unsigned short CRC16_modbus(char *buf, int len)
{
  unsigned int crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (unsigned int)buf[pos]; // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--)
    { // Loop over each bit
      if ((crc & 0x0001) != 0)
      {            // If the LSB is set
        crc >>= 1; // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else         // Else LSB is not set
        crc >>= 1; // Just shift right
    }
  }

  return crc;
}

uint16_t CRC16 (char *nData, uint16_t wLength)
{
static const uint16_t wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

int8_t nTemp;
uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;

}

void serverRead()
{
  unsigned char buftemp[1000];
  int len;
  int start;
  int count = 0;

  millisDelay(300);
  while (simSerial.available())
  { // While data incomming: Read into buffer
    int numBytes = simSerial.available();
    for (int i = 0; i < numBytes; i++)
    {
      buftemp[i] = simSerial.read();
    }
  }

  while (buftemp[count] != 44)
  {
    count++;
  }

  if (count == 4)
  {
    len = ((buftemp[9] - 48) * 1000) + ((buftemp[10] - 48) * 100) + ((buftemp[11] - 48) * 10) + buftemp[12] - 48;
    start = 14;
  }
  else if (count == 3)
  {
    len = ((buftemp[9] - 48) * 100) + ((buftemp[10] - 48) * 10) + buftemp[11] - 48;
    start = 13;
  }
  else if (count == 2)
  {
    len = ((buftemp[9] - 48) * 10) + buftemp[10] - 48;
    start = 12;
  }
  else
  {
    len = buftemp[9] - 48;
    start = 11;
  }

  /********Rigtig kode**********/
  for (int i = 0; i < len; i++)
  {
    simBuffer[i] = buftemp[start + i];
  }
}
