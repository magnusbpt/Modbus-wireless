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
unsigned int bufferLen = 0;

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
void loraFlush();

void setup()
{

  CLKPR = 1 << CLKPCE; // Clock Prescaler Change Enable
  CLKPR = 0;           // Change clock division factor to 1.

  // Serial.begin(9600);
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

  // char modelAndFirmware[20]; // Create message array
  // short model = 0x0001;      // Model nr.
  // short firmware = 0x0001;   // Firmware nr.

  // for (unsigned int i = 0; i < strlen(IMEI); i++)
  // { // Put IMEI into message
  //   modelAndFirmware[i] = IMEI[i];
  // }

  // modelAndFirmware[8] = transactionID++; // Put transactionID into message

  // modelAndFirmware[9] = 0x02; // Put command 02 into message

  // modelAndFirmware[10] = highByte(model); // Put model nr. into message
  // modelAndFirmware[11] = lowByte(model);

  // modelAndFirmware[12] = highByte(firmware); // Put firmware nr. into massage
  // modelAndFirmware[13] = lowByte(firmware);

  // modelAndFirmware[14] = highByte(CRC16_modbus(modelAndFirmware, 14)); // Put CRC into message
  // modelAndFirmware[15] = lowByte(CRC16_modbus(modelAndFirmware, 14));

  // simSerial.println(F("AT+CASEND=0,16")); // Write to send message 16 bytes long
  // millisDelay(200);
  // clrsimBuffer();

  // simSerial.write(modelAndFirmware, sizeof(modelAndFirmware)); // Send model and firmware
  // responseCheck("OK", 2000);
  // clrsimBuffer();

  // simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server
  // responseCheck("49", 2000);
  // clrsimBuffer();

  /**********Ask for slave setup on startup********/
  char msg[15]; // Create message array

  for (int i = 0; i < strlen(IMEI); i++)
  { // Add IMEI to message
    msg[i] = IMEI[i];
  }

  msg[8] = transactionID++; // Add transactionID

  msg[9] = 0x03; // Add command 03

  msg[10] = highByte(CRC16_modbus(msg, 10)); // Add CRC
  msg[11] = lowByte(CRC16_modbus(msg, 10));

  simSerial.println(F("AT+CASEND=0,12")); // Write to send message 12 bytes long
  millisDelay(500);
  clrsimBuffer();

  simSerial.write(msg, sizeof(msg)); // Send message
  responseCheck("OK", 2000);
  clrsimBuffer();

  simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server

  serverRead(); // Read message from server into simBuffer
  clrsimBuffer();

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
  loraSerial.println(F("AT+MODE=TEST"));    // LoRa enter test mode
  loraFlush();

  millisDelay(100);

  loraSerial.println(F("AT+TEST=RFCFG,868,SF12,500,8,10,22,ON,OFF,OFF")); // Set LoRa RF configuration
  loraFlush();

  millisDelay(100);

  loraSerial.println(F("AT+TEST=RXLRPKT")); // Enter LoRa recieve mode
  millisDelay(100);
  loraFlush();

  digitalWrite(BLUE_LED, LOW); // Turn ON LED for message indication

  while (!loraSerial.available()) // Wait for LoRa message or powerOFF && powerStatus()
  {
  }
  delay(100);
  digitalWrite(BLUE_LED, HIGH); // Turn OFF LED for message indication

  loraRead(); // Read LoRa message

  // simSerial.println("AT+CASEND=0,100"); // Need ln when writing to sim module

  // millisDelay(300);

  // simSerial.write(loraBuffer, 100); // Send message to server

  // millisDelay(300);

  // simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server
  // serverRead();                             // Read server message
  // clrsimBuffer();

  // memset(messageBuffer, 0, sizeof messageBuffer);

  char returnID[2];

  returnID[0] = loraBuffer[0];
  returnID[1] = loraBuffer[1];

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

  // unsigned short CRCcheck = (messageBuffer[bufferLen - 2] << 8) | messageBuffer[bufferLen - 1];

  // unsigned short CRCtemp = CRC16_modbus(messageBuffer, bufferLen - 2);

  // unsigned short CRC = (CRCtemp & 0xFF00) | lowByte(~CRCtemp);

  memset(loraBuffer, 0, sizeof loraBuffer);

  // simSerial.println("AT+CASEND=0,40"); // Need ln when writing to sim module

  // millisDelay(300);

  // simSerial.write(messageBuffer, 40); // Send message to server

  // millisDelay(300);

  // simSerial.println(F("AT+CARECV=0,1460")); // Read recieved message from server
  // serverRead();                             // Read server message
  // clrsimBuffer();

  // memset(messageBuffer, 0, sizeof messageBuffer);

  // if (CRC == CRCcheck) //If the checksum match
  // {
  loraSerial.println(F("AT+MODE=TEST"));    // Enter test mode
  loraFlush();

  millisDelay(100);

  loraSerial.println(F("AT+TEST=RFCFG,868,SF12,500,8,10,22,ON,OFF,OFF")); // Set LoRa RF configuration
  loraFlush();

  char loraTX[50] = "AT+TEST=TXLRPKT,\""; // Make array for LoRa message

  loraTX[17] = returnID[0]; // Insert first byte og slave adress
  loraTX[18] = returnID[1]; // Insert second byte og slave adress
  strcat(loraTX, "\"\r\n"); // Insert ", CR and LF to end of message

  millisDelay(1000);

  loraSerial.write(loraTX, strlen(loraTX)); // Write message to LoRa module
  millisDelay(1000);                        // Wait for message to be sent
  loraFlush();

  state = serversend; // Change state to send message to server
  // }
}

void sendToServer()
{
  char serverMessage[500];
  int msgPos = 12;

  if (bufferLen > 4)
  {
    for (unsigned int i = 0; i < strlen(IMEI); i++)
    { // Put IMEI into message
      serverMessage[i] = IMEI[i];
    }

    serverMessage[8] = transactionID++; // Change transactionID for new message

    serverMessage[9] = 0x01; // Add command 04

    serverMessage[10] = 0x00;
    serverMessage[11] = ((bufferLen / 2) - 2) / 6; // Add length of message

    for (unsigned int i = 0; i < (bufferLen / 2) - 2; i++)
    { // Add message
      serverMessage[msgPos++] = messageBuffer[i];
    }

    unsigned short CRC1 = CRC16_modbus(serverMessage, msgPos);

    serverMessage[msgPos] = highByte(CRC1); // Add CRC
    msgPos++;
    serverMessage[msgPos] = lowByte(~CRC1);
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

  while (stop)
  {                                    // Stay in loop while stop is set
    simSerial.println(F("AT+CGREG?")); // Print AT
    start = responseCheck("0,5", 1000);
    if (start)
    { // If start is set, stop while
      stop = 0;
    }
    clrsimBuffer(); // Clear sim response buffer
  }
  start = 0; // Start bit used in simSetup
  stop = 1;  // Set stop for next siminit call

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
  bufferLen = 0;

  while (loraSerial.available())
  { // While data incomming: Read into buffer
    tempBuffer[i] = loraSerial.read();
    i++;
    if (strstr(tempBuffer, "RX \"") > 0)
    {
      break;
    }
  }
  memset(tempBuffer, 0, sizeof tempBuffer);

  while (loraSerial.available())
  { 
    loraBuffer[j] = loraSerial.read();
    if (strstr(loraBuffer, "\""))
    {
      loraBuffer[j] = '\0';
      break;
    }
    bufferLen++;
    j++;
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

void loraFlush()
{ // Read response after sending AT command

  millisDelay(100); // Wait for sim module to respons correctly
  int i = 0;
  char t = 0;

  while (loraSerial.available())
  { // While data incomming: Read into buffer
    t = loraSerial.read();
    i++;
  }
}