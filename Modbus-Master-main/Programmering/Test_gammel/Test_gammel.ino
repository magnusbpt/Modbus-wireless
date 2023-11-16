
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <EEPROM.h>

/********Define pins**********/
#define DTR_E 42
#define PWRKEY 43
#define RED_LED 64
#define GREEN_LED 65
#define BLUE_LED 66
#define Button 67

/********Define swith case numbers**********/
#define slaveread 0
#define serversend 1
#define slavewrite 2
#define poweroff 3

/********Define serial**********/
#define simSerial Serial1
#define modbusSerial Serial2

#define variableBool 1
#define variableInt 2

/********Create arrays**********/
unsigned char* simBuffer = (unsigned char*)malloc(1000 * sizeof(char));
unsigned char* setupMessage = (unsigned char*)malloc(1000 * sizeof(char));
// unsigned char *serverMessage = (unsigned char*)malloc(500 * sizeof(char));
char serverMessage[500];
unsigned char modBuffer[50];  // Buffer to store modbus slave response
char IMEI[8];                // Array to store IMEI number
char CSQ[3];                  // Til at hente NB signalvaerdi.
char IDarray[30];

/********Create global variables**********/
bool slaveSetupCheck = 0;
byte transactionID = -1;
int state = 0;
int ADCValue = 0;
float voltage = 0;
int msgPos = 0;
bool msgFullFlag = 0;

/********Create struct for slave data**********/
struct slaves_info {
  unsigned short analogData[8];
  unsigned char digitalData[8];
  void reset() {
    memset(analogData, 0, sizeof(analogData));
    memset(digitalData, 0, sizeof(digitalData));
  }
};

/********Create struct for write to slave**********/
struct write_Slave {
  unsigned short writeAnalogAdress[8];
  unsigned short writeDigitalAdress[8];
  unsigned short writeAnalogData[8];
  unsigned char writeDigitalData[8];
  unsigned short writeAnalogSmallAdress;
  unsigned short writeAnalogLargeAdress;
  unsigned short writeDigitalSmallAdress;
  unsigned short writeDigitalLargeAdress;
  unsigned short digitalMsgData;
  int writeDigitalCount;
  int writeAnalogCount;
  void reset() {
    memset(writeAnalogAdress, 0, sizeof(writeAnalogAdress));
    memset(writeDigitalAdress, 0, sizeof(writeDigitalAdress));
    memset(writeAnalogData, 0, sizeof(writeAnalogData));
    memset(writeDigitalData, 0, sizeof(writeDigitalData));
    digitalMsgData = 0;
    writeDigitalCount = 0;
    writeAnalogCount = 0;
  }
};

/********Create struct for data to be saved**********/
struct saved_data {
  unsigned short analogAdress[8];
  unsigned short digitalAdress[8];
  unsigned short analogSmallAdress;
  unsigned short analogLargeAdress;
  unsigned short digitalSmallAdress;
  unsigned short digitalLargeAdress;
  unsigned char digitalOldData[8];
  unsigned short analogOldData[8];
  int digitalCount;
  int analogCount;
  unsigned short writeAnalogOldData[8];
  unsigned char writeDigitalOldData[8];
  void reset() {
    memset(analogAdress, 0, sizeof(analogAdress));
    memset(digitalAdress, 0, sizeof(digitalAdress));
  }
};

struct saved_data slaveSaved[60];  //Make struct for 30 slaves

/********Test kode**********/
char testbuf2[12] = { 0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00 };

void setup() {

  CLKPR = 1 << CLKPCE;  // Clock Prescaler Change Enable
  CLKPR = 0;            // Change clock division factor to 1.

  //Begin serial communication
  simSerial.begin(9600);
  modbusSerial.begin(9600);

  //Setup mode of pins on master
  pinMode(DTR_E, OUTPUT);
  pinMode(PWRKEY, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(Button, INPUT);

  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  digitalWrite(PWRKEY, HIGH);  // Set powerkey high so sim module stay on/off
  digitalWrite(DTR_E, LOW);    // Maybe not used?

  millisDelay(3000);  //Start delay

  simSetup();  //Sim module setup

  /**********Send model and firmware***************/

  char modelAndFirmware[20];  //Create message array
  short model = 0x0001;       //Model nr.
  short firmware = 0x0001;    //Firmware nr.
  transactionID++;

  for (int i = 0; i < strlen(IMEI); i++) {  //Put IMEI into message
    modelAndFirmware[i] = IMEI[i];
  }

  modelAndFirmware[8] = transactionID;  //Put transactionID into message

  modelAndFirmware[9] = 0x02;  //Put command 02 into message

  modelAndFirmware[10] = highByte(model);  //Put model nr. into message
  modelAndFirmware[11] = lowByte(model);

  modelAndFirmware[12] = highByte(firmware);  //Put firmware nr. into massage
  modelAndFirmware[13] = lowByte(firmware);

  modelAndFirmware[14] = highByte(CRC16_modbus(modelAndFirmware, 14));  //Put CRC into message
  modelAndFirmware[15] = lowByte(CRC16_modbus(modelAndFirmware, 14));

  simSerial.println(F("AT+CASEND=0,16"));  //Write to send message 16 bytes long
  millisDelay(200);
  clrsimBuffer();

  simSerial.write(modelAndFirmware, sizeof(modelAndFirmware));  // Send model and firmware
  responseCheck("OK", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CARECV=0,1460"));  // Read recieved message from server
  responseCheck("49", 10000);
  clrsimBuffer();


  /**********Ask for slave setup on startup********/
  char msg[15];     // Create message array
  transactionID++;  // Change transactionID for new message

  for (int i = 0; i < strlen(IMEI); i++) {  //Add IMEI to message
    msg[i] = IMEI[i];
  }

  msg[8] = transactionID;  //Add transactionID

  msg[9] = 0x03;  //Add command 03

  msg[10] = highByte(CRC16_modbus(msg, 10));  //Add CRC
  msg[11] = lowByte(CRC16_modbus(msg, 10));

  simSerial.println(F("AT+CASEND=0,12"));  //Write to send message 12 bytes long
  millisDelay(500);
  clrsimBuffer();

  simSerial.write(msg, sizeof(msg));  // Send message
  responseCheck("OK", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CARECV=0,1460"));  // Read recieved message from server

  serverRead();  //Read message from server into simBuffer
  clrsimBuffer();

  /********Test kode**********/
  char testbuf[18] = { 0x01, 0x03, 0x00, 0x02, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x65, 0x6F };

  for (int i = 0; i < 18; i++) {
    simBuffer[i] = testbuf[i];
  }
}

void loop() {

  switch (state) {
    case slaveread:  //Slavesetup and read data from slaves. Also check for change in data.
      modbusRead();
      break;

    case serversend:

      millisDelay(1000);

      sendToServer();  //Send message to server

      simSerial.println(F("AT+CARECV=0,1460"));  // Read recieved message from server

      serverRead();  //Read server message
      clrsimBuffer();

      testbuf2[0] = transactionID;
      testbuf2[10] = highByte(CRC16_modbus(testbuf2, 10));
      testbuf2[11] = lowByte(CRC16_modbus(testbuf2, 10));

      for (int i = 0; i < 12; i++) {
        simBuffer[i] = testbuf2[i];
      }

      // Go to different case depending on response
      if (simBuffer[0] == transactionID) {
        if (simBuffer[1] == 0x00) {  //If read hartbeat, read from slaves again
          state = slaveread;
        } else if (simBuffer[1] == 0x01) {  //If read data, write to slaves
          state = slavewrite;
        } else if (simBuffer[1] == 0x02) {  //If read model and firmware, read from slaves
          state = slaveread;
        } else if (simBuffer[1] == 0x03) {  //If read setup, go to slave setup and read from slaves
          slaveSetupCheck = 0;              //Reset slave setup
          state = slaveread;
        } else if (simBuffer[1] == 0x04) {  //If read master setup, read from slaves
          state = slaveread;
        }
      } else {
        state = serversend;  //Send same message to server if nothing is read
      }
      break;

    case slavewrite:
      modbusWrite();  //Write data response to slaves
      break;

    case poweroff:  //Send power off to server

      char masterData[20];  //Create message array
      transactionID++;

      for (int i = 0; i < strlen(IMEI); i++) {  //Add IMEI
        masterData[i] = IMEI[i];
      }
      masterData[8] = transactionID;

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

      simSerial.println(F("AT+CASEND=0,19"));  //Write to send message 19 bytes long
      millisDelay(500);
      clrsimBuffer();

      simSerial.write(masterData, 19);  //Send message
      responseCheck("OK", 1000);

      simSerial.println(F("AT+CARECV=0,1460"));  //Read recieved message from server
      responseCheck("00", 1000);
      break;
  }

  if (powerStatus() == 0) {  //If power is still on
    state = poweroff;
  }
}

void simSetup() {

  bool start = 0;  // Start bit used in simSetup
  bool stop = 1;   // Stop bit used in simSetup

  while (stop) {                   // Stay in loop while stop is set
    for (int i = 0; i < 5; i++) {  // Check four times for OK response
      simSerial.println(F("AT"));  // Print AT
      start = OKcomcheck(300);     // Check for OK
      if (start) {                 // If start is set, stop while
        stop = 0;
      }
    }
    if (!start) {  // If start is not set sim module is turned of, therefore turn on
      simpow();
    }
  }
  stop = 1;  // Set stop for next siminit call

  clrsimBuffer();  // Clear sim response buffer

  simSerial.println(F("AT+GSN"));  // Sent at command to get IMEI number
  responseCheck("OK", 10000);

  for (int i = 0; i < 7; i++) {  // Put response into IMEI array
    if (i < 1) {
      IMEI[i] = (simBuffer[9] - 48);
    }
    IMEI[i + 1] = ((simBuffer[(i * 2) + 10] - 48) << 4) | (simBuffer[(i * 2) + 11] - 48);
  }
  clrsimBuffer();  // Clear sim response buffer


  simSerial.println(F("AT+CGATT?"));  // Check PS Service
  responseCheck("1", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CNCFG=0,1,\"iot.1nce.net\""));  // Check if connected to the correct APN
  if (responseCheck("OK", 10000) != 1) {                  //If not connected
    simSerial.println(F("AT+CFUN=0"));                    // Set phone functionality to minimal
    responseCheck("OK", 10000);
    clrsimBuffer();

    simSerial.println(F("AT+CGDCONT=1,\"IP\",\"iot.1nce.net\""));  // Define PDP contect: PDP contect identifier, Set to IP, APN
    responseCheck("OK", 10000);
    clrsimBuffer();

    simSerial.println(F("AT+CFUN=1"));  // Set phone funtionality to full
    responseCheck("OK", 10000);
    clrsimBuffer();
  }
  clrsimBuffer();

  simSerial.println(F("AT+COPS?"));  // Check operator. See operator e.g Telia and if connected to NB or CAT-M
  if (responseCheck("9", 5000) != 1) {
    responseCheck("7", 5000);
  }
  clrsimBuffer();

  simSerial.println(F("AT+CGNAPN"));  // Check if connected to the correct APN
  responseCheck("\"iot.1nce.net\"", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CSQ"));  // Check signal quality
  responseCheck("OK", 10000);
  for (int i = 0; i < 2; i++) {  // Put response into IMEI array
    CSQ[i] = simBuffer[i + 7];
  }
  clrsimBuffer();  // Clear sim response buffer

  simSerial.println(F("AT+CNACT=0,1"));  // Activate network
  responseCheck("0,ACTIVE", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CNACT?"));  // Check if network is active to adress
  responseCheck("OK", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CAOPEN=0,0,\"UDP\",\"164.92.164.168\",8080"));  // Open UDP connection/socket
  responseCheck("0,0", 10000);
  clrsimBuffer();
}

void modbusRead() {

  struct slaves_info slave[60];  //Create slave structures

  if (!slaveSetupCheck) {  //Save information of slaves. Will only do after a setup.

    unsigned short dataCount = 0;
    char ID = 0;
    short ADR = 0;
    char VAR = 0;
    short VAL = 0;
    int IDCount = 0;
    memset(IDarray, 0, sizeof(IDarray));

    dataCount = dataCount | simBuffer[2];  //Number of data/adresses to read
    dataCount = dataCount << 8;
    dataCount = dataCount | simBuffer[3];

    for (int i = 0; i < dataCount; i++) {  //Save ID, adresses and initial value
      ID = simBuffer[(i * 6) + 4];
      ADR = simBuffer[(i * 6) + 5];
      ADR = ADR << 8;
      ADR |= simBuffer[(i * 6) + 6];
      VAR = simBuffer[(i * 6) + 7];
      VAL = simBuffer[(i * 6) + 8];
      VAL = VAL << 8;
      VAL |= simBuffer[(i * 6) + 9];

      EEPROM.put(ID, VAL);  //Put initial value into EEPROM

      if (!(strchr(IDarray, ID) > 0)) {  //If ID is not in IDarray, add to array and reset counters
        IDarray[IDCount++] = ID;
        slaveSaved[ID].digitalCount = 0;
        slaveSaved[ID].analogCount = 0;
        slave[ID].reset();
      }

      if (VAR == 1) {  //If variable type is bool, add adress to digital adress
        slaveSaved[ID].digitalAdress[slaveSaved[ID].digitalCount++] = ADR;

      } else {  //Else add adress to analog adress
        slaveSaved[ID].analogAdress[slaveSaved[ID].analogCount++] = ADR;
      }
    }

    clrsimBuffer();  //Clear response buffer

    for (int i = 0; i < strlen(IDarray); i++) {  //Go through IDarray

      if (slaveSaved[IDarray[i]].digitalCount > 0) {  //If there is digital inputs in slave

        slaveSaved[IDarray[i]].digitalSmallAdress = slaveSaved[IDarray[i]].digitalAdress[0];  //Assume first element is smallest
        slaveSaved[IDarray[i]].digitalLargeAdress = slaveSaved[IDarray[i]].digitalAdress[0];  //Assume first element is smallest

        for (int j = 1; j < slaveSaved[IDarray[i]].digitalCount; j++) {
          if (slaveSaved[IDarray[i]].digitalAdress[j] < slaveSaved[IDarray[i]].digitalSmallAdress) {  //Find the smallest digital adress
            slaveSaved[IDarray[i]].digitalSmallAdress = slaveSaved[IDarray[i]].digitalAdress[j];
          }
          if (slaveSaved[IDarray[i]].digitalAdress[j] > slaveSaved[IDarray[i]].digitalLargeAdress) {  //Find the largest digital adress
            slaveSaved[IDarray[i]].digitalLargeAdress = slaveSaved[IDarray[i]].digitalAdress[j];
          }
        }
      }
      if (slaveSaved[IDarray[i]].analogCount > 0) {                                         //If there is digital inputs in slave
        slaveSaved[IDarray[i]].analogSmallAdress = slaveSaved[IDarray[i]].analogAdress[0];  //Assume first element is smallest
        slaveSaved[IDarray[i]].analogLargeAdress = slaveSaved[IDarray[i]].analogAdress[0];  //Assume first element is largest

        for (int j = 1; j < slaveSaved[IDarray[i]].analogCount; j++) {
          if (slaveSaved[IDarray[i]].analogAdress[j] < slaveSaved[IDarray[i]].analogSmallAdress) {  //Find the smallest analog adress
            slaveSaved[IDarray[i]].analogSmallAdress = slaveSaved[IDarray[i]].analogAdress[j];
          }
          if (slaveSaved[IDarray[i]].analogAdress[j] > slaveSaved[IDarray[i]].analogLargeAdress) {  //Find the largest analog adress
            slaveSaved[IDarray[i]].analogLargeAdress = slaveSaved[IDarray[i]].analogAdress[j];
          }
        }
      }
    }
  }
  slaveSetupCheck = 1;  //Setup is done, will only run again if setup is recieved from server.

  //Define ADU size
  char ADU[8];
  int ADUsize = 0;
  short CRCModbus = 0;

  char temp;

  for (int i = 0; i < strlen(IDarray); i++) {  //Go through ID array length

    if (slaveSaved[IDarray[i]].digitalCount > 0) {  //If there are digital intput in slave

      memset(ADU, 0, sizeof ADU);
      ADUsize = 0;
      CRCModbus = 0;

      //Assempling ADU message
      ADU[ADUsize++] = IDarray[i];                                           //Add ID
      ADU[ADUsize++] = 0x02;                                                 //Function code read discrete inputs
      ADU[ADUsize++] = highByte(slaveSaved[IDarray[i]].digitalSmallAdress);  //Add start adress
      ADU[ADUsize++] = lowByte(slaveSaved[IDarray[i]].digitalSmallAdress);
      ADU[ADUsize++] = highByte((slaveSaved[IDarray[i]].digitalLargeAdress - slaveSaved[IDarray[i]].digitalSmallAdress) + 1);  //Add quantity of adress to read
      ADU[ADUsize++] = lowByte((slaveSaved[IDarray[i]].digitalLargeAdress - slaveSaved[IDarray[i]].digitalSmallAdress) + 1);

      //Calculate CRC16 for MODBUS
      CRCModbus = CRC16_modbus(ADU, ADUsize);
      ADU[ADUsize++] = lowByte(CRCModbus);  //Add CRC
      ADU[ADUsize++] = highByte(CRCModbus);

      //Write ADU to MODBUS slave
      modbusSerial.write(ADU, ADUsize);

      slaveRead();  //Read modbus response

      for (int j = 0; j < modBuffer[2]; j++) {  //Put modbus data into digital data array.

        temp = modBuffer[3 + j];

        for (int k = 0; k < modBuffer[2] * 8; k++) {
          slave[IDarray[i]].digitalData[k] = (temp & (1 << k)) >> k;  //Make one byte of data into 8 bytes
        }
      }
    }

    if (slaveSaved[IDarray[i]].analogCount > 0) {  //If there are analog intput in slave

      memset(ADU, 0, sizeof ADU);
      ADUsize = 0;
      CRCModbus = 0;

      //Assempling ADU message
      ADU[ADUsize++] = IDarray[i];                                          //Add ID
      ADU[ADUsize++] = 0x03;                                                //Function code
      ADU[ADUsize++] = highByte(slaveSaved[IDarray[i]].analogSmallAdress);  //Add start adress
      ADU[ADUsize++] = lowByte(slaveSaved[IDarray[i]].analogSmallAdress);
      ADU[ADUsize++] = highByte((slaveSaved[IDarray[i]].analogLargeAdress - slaveSaved[IDarray[i]].analogSmallAdress) + 1);  //Add quantity of adress to read
      ADU[ADUsize++] = lowByte((slaveSaved[IDarray[i]].analogLargeAdress - slaveSaved[IDarray[i]].analogSmallAdress) + 1);

      //Calculate CRC16 for MODBUS
      CRCModbus = CRC16_modbus(ADU, ADUsize);
      ADU[ADUsize++] = lowByte(CRCModbus);
      ADU[ADUsize++] = highByte(CRCModbus);

      //Write ADU to MODBUS slave
      modbusSerial.write(ADU, ADUsize);

      slaveRead();  //Read modbus response

      for (int j = 0; j < (modBuffer[2] / 2); j++) {  //Add bytes of analog data into array
        slave[IDarray[i]].analogData[j] = modBuffer[(2 * j) + 3];
        slave[IDarray[i]].analogData[j] = slave[IDarray[i]].analogData[j] << 8;
        slave[IDarray[i]].analogData[j] |= modBuffer[(2 * j) + 4];
      }
    }

    millisDelay(300);
  }

  clrmodBuffer();


/****************** Make message to server **************************/
  short CRCSim = 0;  // To hold CRC
  msgPos = 12;
  bool noNewData = 0;
  static int totalAdress = 0;
  transactionID++;  //Increment transaction ID before every message
  memset(serverMessage, 0, sizeof serverMessage);

  for (int i = 0; i < strlen(IDarray); i++) {  //For length of ID array

    if (slaveSaved[IDarray[i]].digitalCount > 0) {                                                                                                   //If digital data has to be sent
      for (int j = 0; j < (slaveSaved[IDarray[i]].digitalLargeAdress - slaveSaved[IDarray[i]].digitalSmallAdress) + 1; j++) {                        //For number of adress' in slave
        if (arraycheck(slaveSaved[IDarray[i]].digitalAdress, slaveSaved[IDarray[i]].digitalSmallAdress + j, slaveSaved[IDarray[i]].digitalCount)) {  //If the adress in in the digital adress array
          if (slave[IDarray[i]].digitalData[j] != slaveSaved[IDarray[i]].digitalOldData[j]) {                                                        //If data is not the same as old data
            noNewData = 0;
            serverMessage[msgPos++] = IDarray[i];                                               //Add ID to message
            serverMessage[msgPos++] = highByte(slaveSaved[IDarray[i]].digitalSmallAdress + j);  //Add adress to message
            serverMessage[msgPos++] = lowByte(slaveSaved[IDarray[i]].digitalSmallAdress + j);
            serverMessage[msgPos++] = 0x00;                                               //Position filler since only one byte of data is needed for digital data
            serverMessage[msgPos++] = slave[IDarray[i]].digitalData[j];                   //Add digital data
            slaveSaved[IDarray[i]].digitalOldData[j] = slave[IDarray[i]].digitalData[j];  //Put new data into old data.
          } else {
            noNewData = 1;
          }
        }
        if (!noNewData) {
          totalAdress++;  //Increment quantity of adress'
        }
      }
    }
    if (slaveSaved[IDarray[i]].analogCount > 0) {                                                                                                 //If analog data has to be sent
      for (int j = 0; j < (slaveSaved[IDarray[i]].analogLargeAdress - slaveSaved[IDarray[i]].analogSmallAdress) + 1; j++) {                       //For number of adress' in slave
        if (arraycheck(slaveSaved[IDarray[i]].analogAdress, slaveSaved[IDarray[i]].analogSmallAdress + j, slaveSaved[IDarray[i]].analogCount)) {  //If the adress in in the analog adress array
          if ((slave[IDarray[i]].analogData[j] != slaveSaved[IDarray[i]].analogOldData[j])) {                                                     //If data is not the same as old data
            noNewData = 0;
            serverMessage[msgPos++] = IDarray[i];                                              //Add ID to message
            serverMessage[msgPos++] = highByte(slaveSaved[IDarray[i]].analogSmallAdress + j);  //Add adress to message
            serverMessage[msgPos++] = lowByte(slaveSaved[IDarray[i]].analogSmallAdress + j);
            serverMessage[msgPos++] = highByte(slave[IDarray[i]].analogData[j]);  //Add analog data
            serverMessage[msgPos++] = lowByte(slave[IDarray[i]].analogData[j]);
            slaveSaved[IDarray[i]].analogOldData[j] = slave[IDarray[i]].analogData[j];  //Put new data into old data.
          } else {
            noNewData = 1;
          }
        }
        if (!noNewData) {
          totalAdress++;  //Increment quantity of adress'
        }
      }
    }
  }

  if (totalAdress > 0) {  //If there are adress data to be send

    for (int i = 0; i < strlen(IMEI); i++) {  //Add IMEI to message
      serverMessage[i] = IMEI[i];
    }

    serverMessage[8] = transactionID;  //Add transaction ID

    serverMessage[9] = 0x01;  //Add command 01 for data send

    serverMessage[10] = highByte(totalAdress);  //Add number of data
    serverMessage[11] = lowByte(totalAdress);

    short CRC2 = CRC16_modbus(serverMessage, msgPos);

    // Find CRC from message
    serverMessage[msgPos] = highByte(CRC2);  //Put CRC into message
    msgPos++;
    serverMessage[msgPos] = lowByte(~CRC2);

    state = serversend;  //Change state to send message

  } else {  // If no data to be sent

    for (int i = 0; i < strlen(IMEI); i++) {  //Add IMEI to message
      serverMessage[i] = IMEI[i];
    }

    serverMessage[8] = transactionID;  //Add transaction ID

    msgPos = 9;

    serverMessage[msgPos++] = 0x00;  //Add command 01 for data send

    short CRC2 = CRC16_modbus(serverMessage, msgPos);

    // Find CRC from message
    serverMessage[msgPos] = highByte(CRC2);  //Put CRC into message
    msgPos++;
    serverMessage[msgPos] = lowByte(CRC2);

    state = serversend;  //Change state to send message
  }
}

void sendToServer() {

  char sendLength[20] = "AT+CASEND=0,";  // To hold at command that sends size of data to be send
  char msgLength[20];                    // Hold size of message

  sprintf(msgLength, "%u", msgPos + 1);  // Add message size to end of AT command
  strcat(sendLength, msgLength);

  simSerial.write(sendLength, sizeof(sendLength));  // Send AT command
  simSerial.println();                              // Need ln when writing to sim module

  millisDelay(300);

  simSerial.write(serverMessage, msgPos + 1);  // Send message to server
}

void modbusWrite() {  //Write data to slaves

  struct write_Slave writeSlave[30];

  //Define ADU size
  char ADU[30];
  int ADUsize = 0;
  short CRC = 0;
  bool dataFlag = 0;

  short temp;

  unsigned short dataCount = 0;
  char ID = 0;
  short ADR = 0;
  char VAR = 0;
  char VAL = 0;
  int IDCount = 0;
  char writeIDarray[30];

  dataCount = dataCount | simBuffer[2];  //Number of data/adresses to read
  dataCount = dataCount << 8;
  dataCount = dataCount | simBuffer[3];

  for (int i = 0; i < (dataCount); i++) {  //Save ID and adress'
    ID = simBuffer[(i * 6) + 4];
    ADR = simBuffer[(i * 6) + 5];
    ADR = ADR << 8;
    ADR |= simBuffer[(i * 6) + 6];
    VAR = simBuffer[(i * 6) + 9];
    if (!(strchr(writeIDarray, ID) > 0)) {   //If ID is not in ID array
      writeIDarray[IDCount++] = ID;          //Add ID to ID array
      writeSlave[ID].writeDigitalCount = 0;  //Reset counter
      writeSlave[ID].writeAnalogCount = 0;   //Reset counter
      writeSlave[ID].reset();                //Reset
    }

    if (VAR == 0x01) {                                                              //If data is digital
      writeSlave[ID].writeDigitalAdress[writeSlave[ID].writeDigitalCount++] = ADR;  //Add adress to digital adress array
      writeSlave[ID].writeDigitalData[ADR] = simBuffer[(i * 6) + 8];                //Add data to be written to slave
    } else {
      writeSlave[ID].writeAnalogAdress[writeSlave[ID].writeAnalogCount++] = ADR;  //Add adress to analog adress array
      writeSlave[ID].writeAnalogData[ADR] = simBuffer[(i * 6) + 7];               //Add data to be written
      writeSlave[ID].writeAnalogData[ADR] = writeSlave[ID].writeAnalogData[ADR] << 8;
      writeSlave[ID].writeAnalogData[ADR] |= simBuffer[(i * 6) + 8];  //Add data to be written
    }
  }

  clrsimBuffer();

  for (int i = 0; i < strlen(writeIDarray); i++) {  //For length of ID array

    if (writeSlave[writeIDarray[i]].writeDigitalCount > 0) {                                                    //If digital data has to be written
      writeSlave[writeIDarray[i]].writeDigitalSmallAdress = writeSlave[writeIDarray[i]].writeDigitalAdress[0];  //Assume first element is smallest
      writeSlave[writeIDarray[i]].writeDigitalLargeAdress = writeSlave[writeIDarray[i]].writeDigitalAdress[0];  //Assume first element is largest

      //Find smallest and largest digital adress
      for (int j = 1; j < writeSlave[writeIDarray[i]].writeDigitalCount; j++) {
        if (writeSlave[writeIDarray[i]].writeDigitalAdress[j] < writeSlave[writeIDarray[i]].writeDigitalSmallAdress) {
          writeSlave[writeIDarray[i]].writeDigitalSmallAdress = writeSlave[writeIDarray[i]].writeDigitalAdress[j];
        }
        if (writeSlave[writeIDarray[i]].writeDigitalAdress[j] > writeSlave[writeIDarray[i]].writeDigitalLargeAdress) {
          writeSlave[writeIDarray[i]].writeDigitalLargeAdress = writeSlave[writeIDarray[i]].writeDigitalAdress[j];
        }
      }
    }
    if (writeSlave[writeIDarray[i]].writeAnalogCount > 0) {                                                   //If analog data has to be written
      writeSlave[writeIDarray[i]].writeAnalogSmallAdress = writeSlave[writeIDarray[i]].writeAnalogAdress[0];  //Assume first element is smallest
      writeSlave[writeIDarray[i]].writeAnalogLargeAdress = writeSlave[writeIDarray[i]].writeAnalogAdress[0];  //Assume first element is largest

      //Find smallest and largest analog adress
      for (int j = 1; j < writeSlave[writeIDarray[i]].writeAnalogCount; j++) {
        if (writeSlave[writeIDarray[i]].writeAnalogAdress[j] < writeSlave[writeIDarray[i]].writeAnalogSmallAdress) {
          writeSlave[writeIDarray[i]].writeAnalogSmallAdress = writeSlave[writeIDarray[i]].writeAnalogAdress[j];
        }
        if (writeSlave[writeIDarray[i]].writeAnalogAdress[j] > writeSlave[writeIDarray[i]].writeAnalogLargeAdress) {
          writeSlave[writeIDarray[i]].writeAnalogLargeAdress = writeSlave[writeIDarray[i]].writeAnalogAdress[j];
        }
      }
    }

    writeSlave[writeIDarray[i]].digitalMsgData = 0;
    //Omregner bytes til bits om tilføjer nyt og gamle data til beskeden
    for (int j = 0; j < (writeSlave[writeIDarray[i]].writeDigitalLargeAdress - writeSlave[writeIDarray[i]].writeDigitalSmallAdress) + 1; j++) {
      dataFlag = 0;
      temp = 0;
      for (int k = 0; k < writeSlave[writeIDarray[i]].writeDigitalCount; k++) {
        if (writeSlave[writeIDarray[i]].writeDigitalAdress[k] == writeSlave[writeIDarray[i]].writeDigitalSmallAdress + j) {
          temp = temp | writeSlave[writeIDarray[i]].writeDigitalData[writeSlave[writeIDarray[i]].writeDigitalSmallAdress + j];
          writeSlave[writeIDarray[i]].digitalMsgData = writeSlave[writeIDarray[i]].digitalMsgData | (temp << j);
          slaveSaved[writeIDarray[i]].writeDigitalOldData[writeSlave[writeIDarray[i]].writeDigitalSmallAdress + j] = writeSlave[writeIDarray[i]].writeDigitalData[writeSlave[writeIDarray[i]].writeDigitalSmallAdress + j];
          dataFlag = 1;
        }
      }
      if (!dataFlag) {
        temp = temp | slaveSaved[writeIDarray[i]].writeDigitalOldData[writeSlave[writeIDarray[i]].writeDigitalSmallAdress + j];
        writeSlave[writeIDarray[i]].digitalMsgData = writeSlave[writeIDarray[i]].digitalMsgData | (temp << j);
      }
    }

    if (writeSlave[writeIDarray[i]].writeDigitalCount > 0) {

      memset(ADU, 0, sizeof ADU);
      ADUsize = 0;
      CRC = 0;

      //Assempling ADU message
      ADU[ADUsize++] = writeIDarray[i];
      ADU[ADUsize++] = 0x0F;  //Function code
      ADU[ADUsize++] = highByte(writeSlave[writeIDarray[i]].writeDigitalSmallAdress);
      ADU[ADUsize++] = lowByte(writeSlave[writeIDarray[i]].writeDigitalSmallAdress);
      ADU[ADUsize++] = highByte((writeSlave[writeIDarray[i]].writeDigitalLargeAdress - writeSlave[writeIDarray[i]].writeDigitalSmallAdress) + 1);
      ADU[ADUsize++] = lowByte((writeSlave[writeIDarray[i]].writeDigitalLargeAdress - writeSlave[writeIDarray[i]].writeDigitalSmallAdress) + 1);
      if ((writeSlave[writeIDarray[i]].writeDigitalLargeAdress - writeSlave[writeIDarray[i]].writeDigitalSmallAdress) + 1 > 8) {
        ADU[ADUsize++] = 0x02;
        ADU[ADUsize++] = lowByte(writeSlave[writeIDarray[i]].digitalMsgData);
        ADU[ADUsize++] = highByte(writeSlave[writeIDarray[i]].digitalMsgData);
      } else {
        ADU[ADUsize++] = 0x01;
        ADU[ADUsize++] = lowByte(writeSlave[writeIDarray[i]].digitalMsgData);
      }

      //Calculate CRC16 for MODBUS
      CRC = CRC16_modbus(ADU, ADUsize);
      ADU[ADUsize++] = lowByte(CRC);
      ADU[ADUsize++] = highByte(CRC);

      // ADU to MODBUS slave
      modbusSerial.write(ADU, ADUsize);

      millisDelay(300);
    }

    if (writeSlave[writeIDarray[i]].writeAnalogCount > 0) {
      memset(ADU, 0, sizeof ADU);
      ADUsize = 0;
      CRC = 0;

      //Assempling ADU message
      ADU[ADUsize++] = writeIDarray[i];
      ADU[ADUsize++] = 0x10;  //Function code
      ADU[ADUsize++] = highByte(writeSlave[writeIDarray[i]].writeAnalogSmallAdress);
      ADU[ADUsize++] = lowByte(writeSlave[writeIDarray[i]].writeAnalogSmallAdress);
      ADU[ADUsize++] = highByte((writeSlave[writeIDarray[i]].writeAnalogLargeAdress - writeSlave[writeIDarray[i]].writeAnalogSmallAdress) + 1);
      ADU[ADUsize++] = lowByte((writeSlave[writeIDarray[i]].writeAnalogLargeAdress - writeSlave[writeIDarray[i]].writeAnalogSmallAdress) + 1);
      ADU[ADUsize++] = ((writeSlave[writeIDarray[i]].writeAnalogLargeAdress - writeSlave[writeIDarray[i]].writeAnalogSmallAdress) + 1) * 2;

      for (int j = 0; j < (writeSlave[writeIDarray[i]].writeAnalogLargeAdress - writeSlave[writeIDarray[i]].writeAnalogSmallAdress) + 1; j++) {
        dataFlag = 0;
        for (int k = 0; k < writeSlave[writeIDarray[i]].writeAnalogCount; k++) {
          if (writeSlave[writeIDarray[i]].writeAnalogAdress[k] == writeSlave[writeIDarray[i]].writeAnalogSmallAdress + j) {
            ADU[ADUsize++] = highByte(writeSlave[writeIDarray[i]].writeAnalogData[writeSlave[writeIDarray[i]].writeAnalogSmallAdress + j]);
            ADU[ADUsize++] = lowByte(writeSlave[writeIDarray[i]].writeAnalogData[writeSlave[writeIDarray[i]].writeAnalogSmallAdress + j]);
            dataFlag = 1;
          }
        }
        if (!dataFlag) {
          ADU[ADUsize++] = highByte(slaveSaved[writeIDarray[i]].writeAnalogOldData[writeSlave[writeIDarray[i]].writeAnalogSmallAdress + j]);
          ADU[ADUsize++] = lowByte(slaveSaved[writeIDarray[i]].writeAnalogOldData[writeSlave[writeIDarray[i]].writeAnalogSmallAdress + j]);
        }
      }

      //Calculate CRC16 for MODBUS
      CRC = CRC16_modbus(ADU, ADUsize);
      ADU[ADUsize++] = lowByte(CRC);
      ADU[ADUsize++] = highByte(CRC);

      // ADU to MODBUS slave
      modbusSerial.write(ADU, ADUsize);

      millisDelay(300);
    }
  }
  state = slaveread;
}


void slaveRead() {  // Read modbus response

  millisDelay(300);  // Wait for slaves to process message

  while (modbusSerial.available()) {          // Stay in while while data available
    int numBytes = modbusSerial.available();  // Find number of bytes to read
    for (int i = 0; i < numBytes; i++) {
      modBuffer[i] = modbusSerial.read();  // Read into modBuffer
    }
  }
}

void simpow() {  // Used to power on or off sim module

  digitalWrite(PWRKEY, LOW);
  millisDelay(1500);
  digitalWrite(PWRKEY, HIGH);
}

bool OKcomcheck(int dly) {  // Same as comcheck, but only for check "OK" response and with no while

  bool check = 0;

  millisDelay(dly);

  if (simSerial.available()) {
    int numBytes = simSerial.available();
    for (int i = 0; i < numBytes; i++) {
      simBuffer[i] = simSerial.read();
    }
  }

  if (strstr((char*)simBuffer, "OK")) {
    check = 1;
  } else {
    check = 0;
  }

  clrsimBuffer();
  return check;
}

void sendSignal() {

  char msg[20];

  for (int i = 0; i < strlen(IMEI); i++) {
    msg[i] = IMEI[i];
  }
  msg[8] = 0x01;

  msg[9] = highByte(0x0001);
  msg[10] = lowByte(0x0001);

  msg[11] = 0x00;

  msg[12] = highByte(0x0002);
  msg[13] = lowByte(0x0002);

  msg[14] = highByte(0x0001);
  msg[15] = lowByte(0x0001);

  msg[16] = lowByte(CRC16_modbus(msg, 16));
  msg[17] = highByte(CRC16_modbus(msg, 16));

  simSerial.println(F("AT+CASEND=0,18"));
  millisDelay(500);
  clrsimBuffer();

  simSerial.write(msg, strlen(msg));  // Check if network is active to adress
  responseCheck("OK", 10000);

  simSerial.println(F("AT+CARECV=0,1460"));  // Read recieved message from server
  responseCheck("00", 10000);
}

int responseCheck(char* c, unsigned int timeout) {  // Check if correct response or ERROR.

  unsigned long timerStart = 0;
  unsigned long timerEnd = 0;
  int check = 0;

  bool msg = 0;
  timerStart = millis();

  while (!msg) {
    simRead();

    if (strstr((char*)simBuffer, c)) {
      check = 1;
      msg = 1;
      digitalWrite(GREEN_LED, HIGH);
      millisDelay(125);
      digitalWrite(GREEN_LED, LOW);
      millisDelay(125);
      digitalWrite(GREEN_LED, HIGH);
      millisDelay(125);
      digitalWrite(GREEN_LED, LOW);
      millisDelay(125);
    } else if (strstr((char*)simBuffer, "ERROR")) {
      check = 2;
      msg = 1;
      digitalWrite(RED_LED, HIGH);
      millisDelay(125);
      digitalWrite(RED_LED, LOW);
      millisDelay(125);
      digitalWrite(RED_LED, HIGH);
      millisDelay(125);
      digitalWrite(RED_LED, LOW);
      millisDelay(125);
    }

    timerEnd = millis();

    if (timerEnd - timerStart > timeout) {
      check = 0;
      msg = 1;
      digitalWrite(BLUE_LED, HIGH);
      millisDelay(125);
      digitalWrite(BLUE_LED, LOW);
      millisDelay(125);
      digitalWrite(BLUE_LED, HIGH);
      millisDelay(125);
      digitalWrite(BLUE_LED, LOW);
      millisDelay(125);
    }
  }

  return check;
}

void simRead() {  // Read response after sending AT command

  millisDelay(500);  // Wait for sim module to respons correctly

  while (simSerial.available()) {  // While data incomming: Read into buffer
    int numBytes = simSerial.available();
    for (int i = 0; i < numBytes; i++) {
      simBuffer[i] = simSerial.read();
    }
  }
}

void serverRead() {
  unsigned char buftemp[1000];
  int len;
  int start;
  int count = 0;

  millisDelay(300);
  while (simSerial.available()) {  // While data incomming: Read into buffer
    int numBytes = simSerial.available();
    for (int i = 0; i < numBytes; i++) {
      buftemp[i] = simSerial.read();
    }
  }

  while (buftemp[count] != 44) {
    count++;
  }

  if (count == 4) {
    len = ((buftemp[9] - 48) * 1000) + ((buftemp[10] - 48) * 100) + ((buftemp[11] - 48) * 10) + buftemp[12] - 48;
    start = 14;
  } else if (count == 3) {
    len = ((buftemp[9] - 48) * 100) + ((buftemp[10] - 48) * 10) + buftemp[11] - 48;
    start = 13;
  } else if (count == 2) {
    len = ((buftemp[9] - 48) * 10) + buftemp[10] - 48;
    start = 12;
  } else {
    len = buftemp[9] - 48;
    start = 11;
  }

  /********Rigtig kode**********/
  for (int i = 0; i < len; i++) {
    simBuffer[i] = buftemp[start + i];
  }
}

bool powerStatus() {
  ADCValue = analogRead(A0);
  voltage = ADCValue * (5.0 / 1023.0);

  if (voltage > 2) {
    return 1;
  } else {
    return 0;
  }
}

void millisDelay(int delayTime) {

  unsigned long time_now = millis();

  while (millis() - time_now < delayTime) {
    //wait.
  }
}

void clrsimBuffer() {  // Clear simBuffer
  memset(simBuffer, 0, sizeof simBuffer);
}

void clrmodBuffer() {  // Clear modBuffer
  memset(modBuffer, 0, sizeof modBuffer);
}

unsigned short CRC16_modbus(char* buf, int len) {
  unsigned int crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (unsigned int)buf[pos];  // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {  // Loop over each bit
      if ((crc & 0x0001) != 0) {    // If the LSB is set
        crc >>= 1;                  // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else        // Else LSB is not set
        crc >>= 1;  // Just shift right
    }
  }

  return crc;
}

bool arraycheck(unsigned short* arr, short num, int len) {

  for (int i = 0; i < len; i++) {
    if (arr[i] == num) {
      return 1;
    }
  }
  return 0;
}
