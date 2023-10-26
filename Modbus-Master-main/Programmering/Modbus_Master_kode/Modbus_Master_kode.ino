
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

#define slaveread 0
#define serversend 1
#define slavewrite 2
#define poweroff 3

#define simSerial Serial1
#define modbusSerial Serial2

#define variableBool 1
#define variableInt 2

unsigned char* simBuffer = (unsigned char*)malloc(1000 * sizeof(char));
unsigned char* serverMessage = (unsigned char*)malloc(2000 * sizeof(char));
unsigned char modBuffer[50];  // Buffer to store modbus slave response
char IMEI[20];                // Array to store IMEI number
char CSQ[3];                  // Til at hente NB signalvaerdi.
char IDarray[30];
bool slaveSetupCheck = 0;
byte keyNumber = -1;

int state = 0;
int ADCValue = 0;
float voltage = 0;

struct slaves_info {
  unsigned short analogData[16];
  unsigned char digitalData[16];
  void reset() {
    memset(analogData, 0, sizeof(analogData));
    memset(digitalData, 0, sizeof(digitalData));
  }
};

struct write_Slave {
  unsigned short writeAnalogAdress[16];
  unsigned short writeDigitalAdress[16];
  unsigned short writeAnalogData[16];
  unsigned short writeAnalogSmallAdress;
  unsigned short writeAnalogLargeAdress;
  unsigned char writeDigitalData[16];
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

struct saved_data {
  unsigned short analogAdress[16];
  unsigned short digitalAdress[16];
  unsigned short analogSmallAdress;
  unsigned short analogLargeAdress;
  unsigned short digitalSmallAdress;
  unsigned short digitalLargeAdress;
  unsigned char digitalOldData[16];
  unsigned short analogOldData[16];
  int digitalCount;
  int analogCount;
  unsigned short writeAnalogOldData[16];
  unsigned char writeDigitalOldData[16];
  void reset() {
    memset(analogAdress, 0, sizeof(analogAdress));
    memset(digitalAdress, 0, sizeof(digitalAdress));
  }
};

struct saved_data slaveSaved[30];

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

  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  digitalWrite(PWRKEY, HIGH);
  digitalWrite(DTR_E, LOW);

  simSetup();  //Sim module setup

  /**********Send model and firmware***************/

  char modelAndFirmware[16];
  short model = 0x0001;     //Model nr.
  short firmware = 0x0001;  //Firmware nr.

  for (int i = 0; i < strlen(IMEI); i++) {
    modelAndFirmware[i] = IMEI[i];
  }
  modelAndFirmware[8] = 0x02;

  modelAndFirmware[9] = highByte(model);
  modelAndFirmware[10] = lowByte(model);

  modelAndFirmware[11] = highByte(firmware);
  modelAndFirmware[12] = lowByte(firmware);

  modelAndFirmware[13] = lowByte(CRC16_modbus(modelAndFirmware, 13));
  modelAndFirmware[14] = highByte(CRC16_modbus(modelAndFirmware, 13));

  simSerial.println(F("AT+CASEND=0,15"));
  responseCheck("OK", 10000);
  clrsimBuffer();

  simSerial.write(modelAndFirmware, strlen(modelAndFirmware));  // Check if network is active to adress
  responseCheck("OK", 10000);

  simSerial.println(F("AT+CARECV=0,1460"));  // Read recieved message from server
  responseCheck("00", 10000);

  /************************************************/

  /**********Ask for slave setup on startup********/
  char msg[15];

  for (int i = 0; i < strlen(IMEI); i++) {
    msg[i] = IMEI[i];
  }
  msg[8] = 0x03;

  msg[9] = lowByte(CRC16_modbus(msg, 9));
  msg[10] = highByte(CRC16_modbus(msg, 9));

  simSerial.println(F("AT+CASEND=0,11"));
  responseCheck("OK", 10000);
  clrsimBuffer();

  simSerial.write(msg, strlen(msg));  // Check if network is active to adress
  responseCheck("OK", 10000);

  simSerial.println(F("AT+CARECV=0,1460"));  // Read recieved message from server

  serverRead();
  /************************************************/
}

void loop() {

  switch (state) {
    case slaveread:  //Slavesetup and read data from slaves. Also check for change in data.
      modbusRead();
      break;

    case serversend:

      sendToServer();  //Send message to server

      simSerial.println(F("AT+CARECV=0,1460"));  // Read recieved message from server

      serverRead();  //Read server message

      if (simBuffer[0] == keyNumber) {
        if (simBuffer[1] == 0x00) {  //Go to different case depending on response
          state = slaveread;
        } else if (simBuffer[1] == 0x01) {
          state = slavewrite;
        } else if (simBuffer[1] == 0x02) {
          state = slaveread;
        } else if (simBuffer[1] == 0x03) {
          slaveSetupCheck = 0;
          state = slaveread;
        } else if (simBuffer[1] == 0x04) {
          state = slaveread;
        }
      } else {
        state = serversend;
      }

    case slavewrite:
      modbusWrite();  //Write data response to slaves
      break;

    case poweroff:  //Send power off to server

      char masterData[20];

      for (int i = 0; i < strlen(IMEI); i++) {
        masterData[i] = IMEI[i];
      }
      masterData[8] = 0x01;

      masterData[9] = highByte(0x0001);
      masterData[10] = lowByte(0x0001);

      masterData[11] = 0x00;

      masterData[12] = highByte(0x0001);
      masterData[13] = lowByte(0x0001);

      masterData[14] = highByte(0x0001);
      masterData[15] = lowByte(0x0001);

      masterData[16] = lowByte(CRC16_modbus(masterData, 16));
      masterData[17] = highByte(CRC16_modbus(masterData, 16));

      simSerial.println(F("AT+CASEND=0,18"));
      responseCheck("OK", 10000);
      clrsimBuffer();

      simSerial.write(masterData, strlen(masterData));  // Check if network is active to adress
      responseCheck("OK", 10000);

      simSerial.println(F("AT+CARECV=0,1460"));  // Read recieved message from server
      responseCheck("00", 10000);
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
    for (int i = 0; i < 4; i++) {  // Check four times for OK response
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

  clrsimBuffer();

  simSerial.println(F("AT+GSN"));  // Sent at command to get IMEI number
  responseCheck("OK", 10000);

  for (int i = 0; i < 8; i++) {  // Put response into IMEI array
    IMEI[i] = (simBuffer[(i * 2) + 9] - 48) << 4;
    if (i < 7) {
      IMEI[i] = IMEI[i] | (simBuffer[(i * 2) + 10] - 48);
    }
  }
  clrsimBuffer();  // Clear sim response buffer

  simSerial.println(F("AT+CGATT?"));  // Check operator. See operator e.g Telia and if connected to NB or CAT-M
  responseCheck("1", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+COPS?"));  // Check operator. See operator e.g Telia and if connected to NB or CAT-M
  if (responseCheck("9", 5000) != 1) {
    responseCheck("7", 5000);
  }
  clrsimBuffer();

  simSerial.println(F("AT+CGNAPN"));
  responseCheck("1,\"iot.1nce.net\"", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CSQ"));  // Check signal quality
  responseCheck("OK", 10000);
  for (int i = 0; i < 2; i++) {  // Put response into IMEI array
    CSQ[i] = simBuffer[i + 7];
  }
  clrsimBuffer();  // Clear sim response buffer

  simSerial.println(F("AT+CNCFG=0,1,\"iot.1nce.net\""));  // Check if connected to the correct APN
  responseCheck("OK", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CNACT=0,1"));  // Activate network
  responseCheck("0,ACTIVE", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CNACT?"));  // Check if network is active to adress
  responseCheck("OK", 10000);
  clrsimBuffer();

  simSerial.println(F("AT+CAOPEN=0,0,\"UDP\",\"207.154.251.171\",8080"));  // Open UDP connection/socket
  responseCheck("OK", 10000);
  clrsimBuffer();
}

void modbusRead() {

  struct slaves_info slave[30];  //Create slave structures

  if (!slaveSetupCheck) {  //Save information of slaves. Will only do after a setup.

    unsigned short dataCount = 0;
    char ID = 0;
    short ADR = 0;
    char VAR = 0;
    short VAL = 0;
    int IDCount = 0;
    memset(IDarray, 0, sizeof(IDarray));

    dataCount = dataCount | simBuffer[1];  //Number of data/adresses to read
    dataCount = dataCount << 8;
    dataCount = dataCount | simBuffer[2];

    for (int i = 0; i < (dataCount); i++) {  //Save ID, adresses and initial value
      ID = simBuffer[(i * 6) + 3];
      ADR = simBuffer[(i * 6) + 4];
      ADR = ADR << 8;
      ADR |= simBuffer[(i * 6) + 5];
      VAR = simBuffer[(i * 6) + 6];
      VAL = simBuffer[(i * 6) + 7];
      VAL = VAL << 8;
      VAL |= simBuffer[(i * 6) + 8];

      EEPROM.put(ID, VAL);

      if (!(strchr(IDarray, ID) > 0)) {
        IDarray[IDCount++] = ID;
        slaveSaved[ID].digitalCount = 0;
        slaveSaved[ID].analogCount = 0;
        slave[ID].reset();
      }

      if (VAR == 1) {
        slaveSaved[ID].digitalAdress[slaveSaved[ID].digitalCount++] = ADR;

      } else {
        slaveSaved[ID].analogAdress[slaveSaved[ID].analogCount++] = ADR;
      }
    }
    
    clrsimBuffer();

    for (int i = 0; i < strlen(IDarray); i++) {

      if (slaveSaved[IDarray[i]].digitalCount > 0) {

        slaveSaved[IDarray[i]].digitalSmallAdress = slaveSaved[IDarray[i]].digitalAdress[0];  //Assume first element is smallest
        slaveSaved[IDarray[i]].digitalLargeAdress = slaveSaved[IDarray[i]].digitalAdress[0];  //Assume first element is smallest

        for (int j = 1; j < slaveSaved[IDarray[i]].digitalCount; j++) {
          if (slaveSaved[IDarray[i]].digitalAdress[j] < slaveSaved[IDarray[i]].digitalSmallAdress) {
            slaveSaved[IDarray[i]].digitalSmallAdress = slaveSaved[IDarray[i]].digitalAdress[j];
          }
          if (slaveSaved[IDarray[i]].digitalAdress[j] > slaveSaved[IDarray[i]].digitalLargeAdress) {
            slaveSaved[IDarray[i]].digitalLargeAdress = slaveSaved[IDarray[i]].digitalAdress[j];
          }
        }
      }
      if (slaveSaved[IDarray[i]].analogCount > 0) {
        slaveSaved[IDarray[i]].analogSmallAdress = slaveSaved[IDarray[i]].analogAdress[0];  //Assume first element is smallest
        slaveSaved[IDarray[i]].analogLargeAdress = slaveSaved[IDarray[i]].analogAdress[0];  //Assume first element is largest

        for (int j = 1; j < slaveSaved[IDarray[i]].analogCount; j++) {
          if (slaveSaved[IDarray[i]].analogAdress[j] < slaveSaved[IDarray[i]].analogSmallAdress) {
            slaveSaved[IDarray[i]].analogSmallAdress = slaveSaved[IDarray[i]].analogAdress[j];
          }
          if (slaveSaved[IDarray[i]].analogAdress[j] > slaveSaved[IDarray[i]].analogLargeAdress) {
            slaveSaved[IDarray[i]].analogLargeAdress = slaveSaved[IDarray[i]].analogAdress[j];
          }
        }
      }
    }
  }
  slaveSetupCheck = 1;

  //Define ADU size
  char ADU[8];
  int ADUsize = 0;
  short CRCModbus = 0;

  char temp;

  for (int i = 0; i < strlen(IDarray); i++) {

    if (slaveSaved[IDarray[i]].digitalCount > 0) {

      memset(ADU, 0, sizeof ADU);
      ADUsize = 0;
      CRCModbus = 0;

      //Assempling ADU message
      ADU[ADUsize++] = IDarray[i];
      ADU[ADUsize++] = 0x02;  //Function code
      ADU[ADUsize++] = highByte(slaveSaved[IDarray[i]].digitalSmallAdress);
      ADU[ADUsize++] = lowByte(slaveSaved[IDarray[i]].digitalSmallAdress);
      ADU[ADUsize++] = highByte((slaveSaved[IDarray[i]].digitalLargeAdress - slaveSaved[IDarray[i]].digitalSmallAdress) + 1);
      ADU[ADUsize++] = lowByte((slaveSaved[IDarray[i]].digitalLargeAdress - slaveSaved[IDarray[i]].digitalSmallAdress) + 1);

      //Calculate CRC16 for MODBUS
      CRCModbus = CRC16_modbus(ADU, ADUsize);
      ADU[ADUsize++] = lowByte(CRCModbus);
      ADU[ADUsize++] = highByte(CRCModbus);

      //Write ADU to MODBUS slave
      modbusSerial.write(ADU, ADUsize);

      slaveRead();

      for (int j = 0; j < modBuffer[2]; j++) {

        temp = modBuffer[3 + j];

        for (int k = 0; k < modBuffer[2] * 8; k++) {
          slave[IDarray[i]].digitalData[k] = (temp & (1 << k)) >> k;
        }
      }
    }

    if (slaveSaved[IDarray[i]].analogCount > 0) {

      memset(ADU, 0, sizeof ADU);
      ADUsize = 0;
      CRCModbus = 0;

      //Assempling ADU message
      ADU[ADUsize++] = IDarray[i];
      ADU[ADUsize++] = 0x03;  //Function code
      ADU[ADUsize++] = highByte(slaveSaved[IDarray[i]].analogSmallAdress);
      ADU[ADUsize++] = lowByte(slaveSaved[IDarray[i]].analogSmallAdress);
      ADU[ADUsize++] = highByte((slaveSaved[IDarray[i]].analogLargeAdress - slaveSaved[IDarray[i]].analogSmallAdress) + 1);
      ADU[ADUsize++] = lowByte((slaveSaved[IDarray[i]].analogLargeAdress - slaveSaved[IDarray[i]].analogSmallAdress) + 1);

      //Calculate CRC16 for MODBUS
      CRCModbus = CRC16_modbus(ADU, ADUsize);
      ADU[ADUsize++] = lowByte(CRCModbus);
      ADU[ADUsize++] = highByte(CRCModbus);

      //Write ADU to MODBUS slave
      modbusSerial.write(ADU, ADUsize);

      slaveRead();

      for (int j = 0; j < (modBuffer[2] / 2); j++) {
        slave[IDarray[i]].analogData[j] = modBuffer[(2 * j) + 3];
        slave[IDarray[i]].analogData[j] = slave[IDarray[i]].analogData[j] << 8;
        slave[IDarray[i]].analogData[j] |= modBuffer[(2 * j) + 4];
      }
    }
  }

  clrmodBuffer();

  short CRCSim = 0;  // To hold CRC
  int msgPos = 19;
  bool noNewData = 0;
  short totalAdress = 0;
  keyNumber++;
  memset(serverMessage, 0, sizeof serverMessage);

  for (int i = 0; i < 15; i++) {
    serverMessage[i] = IMEI[i];
  }

  serverMessage[15] = keyNumber;

  serverMessage[16] = 0x01;

  for (int i = 0; i < strlen(IDarray); i++) {

    if (slaveSaved[IDarray[i]].digitalCount > 0) {
      for (int j = 0; j < (slaveSaved[IDarray[i]].digitalLargeAdress - slaveSaved[IDarray[i]].digitalSmallAdress) + 1; j++) {
        if (arraycheck(slaveSaved[IDarray[i]].digitalAdress, slaveSaved[IDarray[i]].digitalSmallAdress + j, slaveSaved[IDarray[i]].digitalCount)) {
          if (slave[IDarray[i]].digitalData[j] != slaveSaved[IDarray[i]].digitalOldData[j]) {
            noNewData = 0;
            serverMessage[msgPos++] = IDarray[i];
            serverMessage[msgPos++] = highByte(slaveSaved[IDarray[i]].digitalSmallAdress + j);
            serverMessage[msgPos++] = lowByte(slaveSaved[IDarray[i]].digitalSmallAdress + j);
            serverMessage[msgPos++] = 0x00;
            serverMessage[msgPos++] = slave[IDarray[i]].digitalData[j];
            slaveSaved[IDarray[i]].digitalOldData[j] = slave[IDarray[i]].digitalData[j];
          } else {
            noNewData = 1;
          }
        }
        if (!noNewData) {
          totalAdress++;
        }
      }
    }
    if (slaveSaved[IDarray[i]].analogCount > 0) {
      for (int j = 0; j < (slaveSaved[IDarray[i]].analogLargeAdress - slaveSaved[IDarray[i]].analogSmallAdress) + 1; j++) {
        if (arraycheck(slaveSaved[IDarray[i]].analogAdress, slaveSaved[IDarray[i]].analogSmallAdress + j, slaveSaved[IDarray[i]].analogCount)) {
          if ((slave[IDarray[i]].analogData[j] != slaveSaved[IDarray[i]].analogOldData[j])) {
            noNewData = 0;
            serverMessage[msgPos++] = IDarray[i];
            serverMessage[msgPos++] = highByte(slaveSaved[IDarray[i]].analogSmallAdress + j);
            serverMessage[msgPos++] = lowByte(slaveSaved[IDarray[i]].analogSmallAdress + j);
            serverMessage[msgPos++] = highByte(slave[IDarray[i]].analogData[j]);
            serverMessage[msgPos++] = lowByte(slave[IDarray[i]].analogData[j]);
            slaveSaved[IDarray[i]].analogOldData[j] = slave[IDarray[i]].analogData[j];
          } else {
            noNewData = 1;
          }
        }
        if (!noNewData) {
          totalAdress++;
        }
      }
    }
  }

  if (totalAdress > 0) {
    serverMessage[17] = highByte(totalAdress);
    serverMessage[18] = lowByte(totalAdress);

    CRCSim = CRC16_modbus((char*)serverMessage, strlen((char*)serverMessage));  // Find CRC from message

    serverMessage[msgPos++] = highByte(CRCSim);
    serverMessage[msgPos++] = lowByte(CRCSim);

    state = serversend;
  } else {
    state = slaveread;
  }
}

void sendToServer() {

  char sendLength[20] = "AT+CASEND=0,";  // To hold at command that sends size of data to be send
  char msgLength[20];                    // Hold size of message

  sprintf(msgLength, "%u", strlen((char*)serverMessage));  // Add message size to end of AT command
  strcat(sendLength, msgLength);

  simSerial.write(sendLength, strlen(sendLength));  // Send AT command
  simSerial.println();                              // Need ln when writing to sim module

  millisDelay(200);

  simSerial.write((char*)serverMessage, strlen((char*)serverMessage));  // Send message to server
}

void modbusWrite() {

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

  dataCount = dataCount | simBuffer[1];
  dataCount = dataCount << 8;
  dataCount = dataCount | simBuffer[2];

  for (int i = 0; i < (dataCount); i++) {
    ID = simBuffer[(i * 6) + 3];
    ADR = simBuffer[(i * 6) + 4];
    ADR = ADR << 8;
    ADR |= simBuffer[(i * 6) + 5];
    VAR = simBuffer[(i * 6) + 8];
    if (!(strchr(writeIDarray, ID) > 0)) {
      writeIDarray[IDCount++] = ID;
      writeSlave[ID].writeDigitalCount = 0;
      writeSlave[ID].writeAnalogCount = 0;
      writeSlave[ID].reset();
    }

    if (VAR == 0x01) {
      writeSlave[ID].writeDigitalAdress[writeSlave[ID].writeDigitalCount++] = ADR;
      writeSlave[ID].writeDigitalData[ADR] = simBuffer[(i * 6) + 7];
    } else {
      writeSlave[ID].writeAnalogAdress[writeSlave[ID].writeAnalogCount++] = ADR;
      writeSlave[ID].writeAnalogData[ADR] = simBuffer[(i * 6) + 6];
      writeSlave[ID].writeAnalogData[ADR] = writeSlave[ID].writeAnalogData[ADR] << 8;
      writeSlave[ID].writeAnalogData[ADR] |= simBuffer[(i * 6) + 7];
    }
  }

  clrsimBuffer();

  for (int i = 0; i < strlen(writeIDarray); i++) {

    if (writeSlave[writeIDarray[i]].writeDigitalCount > 0) {
      writeSlave[writeIDarray[i]].writeDigitalSmallAdress = writeSlave[writeIDarray[i]].writeDigitalAdress[0];  //Assume first element is smallest
      writeSlave[writeIDarray[i]].writeDigitalLargeAdress = writeSlave[writeIDarray[i]].writeDigitalAdress[0];  //Assume first element is largest

      for (int j = 1; j < writeSlave[writeIDarray[i]].writeDigitalCount; j++) {
        if (writeSlave[writeIDarray[i]].writeDigitalAdress[j] < writeSlave[writeIDarray[i]].writeDigitalSmallAdress) {
          writeSlave[writeIDarray[i]].writeDigitalSmallAdress = writeSlave[writeIDarray[i]].writeDigitalAdress[j];
        }
        if (writeSlave[writeIDarray[i]].writeDigitalAdress[j] > writeSlave[writeIDarray[i]].writeDigitalLargeAdress) {
          writeSlave[writeIDarray[i]].writeDigitalLargeAdress = writeSlave[writeIDarray[i]].writeDigitalAdress[j];
        }
      }
    }
    if (writeSlave[writeIDarray[i]].writeAnalogCount > 0) {
      writeSlave[writeIDarray[i]].writeAnalogSmallAdress = writeSlave[writeIDarray[i]].writeAnalogAdress[0];  //Assume first element is smallest
      writeSlave[writeIDarray[i]].writeAnalogLargeAdress = writeSlave[writeIDarray[i]].writeAnalogAdress[0];  //Assume first element is largest

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
    }
  }
}

void slaveRead() {  // Read modbus response

  millisDelay(100);  // Wait for slaves to process message

  while (modbusSerial.available()) {  // Stay in while while data available

    int numBytes = modbusSerial.available();  // Find number of bytes to read
    for (int i = 0; i < numBytes; i++) {
      modBuffer[i] = modbusSerial.read();  // Read into modBuffer
    }
  }
}

void simpow() {  // Used to power on or off sim module

  digitalWrite(PWRKEY, LOW);
  millisDelay(1000);
  digitalWrite(PWRKEY, HIGH);
}

bool OKcomcheck(int dly) {  // Same as comcheck, but only for check "OK" response and with no while

  bool check = 0;

  millisDelay(dly);

  while (simSerial.available()) {
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
  responseCheck("OK", 10000);
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

  millisDelay(1000);  // Wait for sim module to respons correctly

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

  while(buftemp[count] != 44){
    count ++;
  }

  if (count == 4) {
    len = ((buftemp[9] - 48) * 1000) + ((buftemp[10] - 48) * 100) + ((buftemp[11] - 48) * 10) + buftemp[12] - 48;
    start = 14;
  } else if (count == 3) {
    len = ((buftemp[9] - 48) * 100) + ((buftemp[10] - 48) * 10) + buftemp[11] - 48;
    start = 13;
  } else if (count == 2){
    len = ((buftemp[9] - 48) * 10) + buftemp[10] - 48;
    start = 12;
  } else{
    len = buftemp[9] - 48;
    start = 11;
  }

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

unsigned short CRC16_modbus(char* buf, int len) {  //Find modbus CRC16
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
