#include <math.h>
#include <string.h>
#include <stdlib.h>

// Modbus function code defines
#define FC_readcoils 0x01
#define FC_readdiscreteinputs 0x02
#define FC_readholdingregisters 0x03
#define FC_readinputregisters 0x04
#define FC_writesinglecoil 0x05
#define FC_writesingleregister 0x06
#define FC_writecoils 0x0F
#define FC_writemultipleregisters 0x10

#define RED_LED 64
#define GREEN_LED 65
#define BLUE_LED 66

static byte value[50];      // Array to store isolated modbus slave adress value e.g temerature of adress 1
byte modBuffer[50];         // Buffer to store modbus slave response
byte simMsg[120];           // Array with message from sim module
char simBuffer[120];        // Buffer to store sim module response
char IMEI[20];              // Array to store IMEI number

char model[] = "K";         // Master model
char firmware[] = "13";     // Master firmware version
char extPower[] = "1";      // External power 0 or 1

bool start = 0;             // Start bit used in siminit 
bool stop = 1;              // Stop bit used in siminit
bool slaveReg = 0;          // Slave register bit used in modbusValue to see if there was response from slave

int lengthCounter = 0;      // General counter to count array lengths

byte* dataByte;             // Pointer to array with bytes
short* dataShort;           // Pointer to array with shorts 

void setup() {

  CLKPR = 1 << CLKPCE;      // Clock Prescaler Change Enable
  CLKPR = 0;                // Change clock division factor to 1.

  Serial1.begin(9600);      // Begin serial with sim module on hardware serial 2 with baud rate 9600
  Serial2.begin(9600);       // Begin serial with Modbus baud rate 9600

  pinMode(BLUE_LED, OUTPUT);
  pinMode(43, OUTPUT);       // Set pin 5 on arduino as input for pwr key on sim module
  pinMode(42, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(67, INPUT);

  digitalWrite(43, HIGH);    // Set pin 5 to standard high for pwr key to not activate

  digitalWrite(42, LOW);    // Set pin 5 to standard high for pwr key to not activate

  delay(2000);              // Start delay before siminit. To make sure sim module is properly turned of or on

  siminit();                // Start siminit to turn on sim module and make sure communication works

  getIMEI();                // Read IMEI number
  
  Sim_Connect();            // Connect to s erver and open socket

  clrsimBuffer();

}

void loop() {

  readholdingregisters(1, 0, 1);    // Read holding register af tempurature of slave 1

  dataByte = modbusValue();         // Put the data into databyte

  dataShort = tempConvert(bytesToShort(dataByte, lengthCounter));    // Make the bytes to short and convert it into the correct value for the server to read

  Sim_send(dataShort, lengthCounter);     // Send the input data of slave 1 to the sim module and read the respone

  writecoils(2, 0, 1, relaySimToMod());   // Write relay output to slave 2 from sim module response
  clrmodBuffer();                         // Clear modbus buffer

  // char testmsg[12] = {0x08, 0x60, 0x01, 0x60, 0x40, 0x63, 0x21, 0x48, 0x00, 0x00, 0xC7, 0xB8};

  // Serial1.println(F("AT+CASEND=0,12"));                              // Need ln when writing to sim module
  // simread();
  // clrsimBuffer();

  // Serial1.write(testmsg, 12);  // Send message to server
  // simread();
  // LED_status();
  // clrsimBuffer();

  // Serial1.println(F("AT+CAACK=0")); // Query Send Data Information
  // simread();
  // LED_status();
  // clrsimBuffer();

  // Serial1.println(F("AT+CARECV=0,50"));   // Read recieved message from server
  // simread();
  // LED_status();

  // simReadData();    // Read message into array


  // writeTerminal();                        // Function to normal write to sim module
}

void getIMEI(){                     // Function to get IMEI number
  Serial1.println(F("AT+GSN"));     // Sent at command to get IMEI number
  simread();                        // Read response

  for(int i = 0; i < 15; i++){      // Put response into IMEI array
    IMEI[i] = simBuffer[i + 9];
  }

  clrsimBuffer();                   // Clear sim response buffer
}

byte* relaySimToMod() {             // Function that returns number of relays, represented by 1 bit, that is to be turned on from sim module response

  static byte out[50];
  byte j[10] = {0};
  int bytes = lengthCounter - 10;   // Calculate number of bytes/relays that has to be changed. Only works with 1 input!!!!! Made for specific case

  for (int k = 0; k < (bytes / 8) + ((bytes % 8) != 0); k++) {    // For loop to the nearest highest value e.g 3 relays = 1 byte, 8 relays = 1 byte, 9 relays = 2 bytes.

    if((bytes % 8) < 1){      // If number of bytes is 8, 16, 24... is bytes is 8.
      bytes = 8;
    } else{
      bytes = bytes % 8;      // If bytes mod 8 is not 0, make bytes the mod value
    }
    
    for (int i = 0; i < bytes; i++) {       // Put sim message relay output into array for modbus relay output. Sim relay output = 1 (1 byte), modbus relay output = 0x01 (1 bit).
      j[k] = j[k] | (simMsg[i + 1] << i);   // Convert sim relay status into bytes for modbus output. There will only be one modbus relay byte pr. 8 output bytes from sim.
    }
    
    out[k] = j[k];
  }
  return out;

}

short* bytesToShort(byte* byteArr, int len) {   // Converts an array of bytes to an array of shorts

  static short data[50];

  short sh = 0;
  int k = 0;

  lengthCounter = 0;    // Reset counter for length of short array

  memset(data, 0, sizeof data);

  for (int i = 0; i < (len / 2); i++) {     // For every 2 bytes need 1 short

    sh = 0;

    sh = (byteArr[k] << 8) | byteArr[k + 1];    // Put two bytes into short

    Serial.println(sh);

    k = k + 2;      // Increment k by 2 so it does not use the same byte twice

    data[i] = sh;     // put into output array
    lengthCounter++;  // Count array length
  }

  return data;
}

void Sim_Connect() {                  // Connect to sim server and open UDP socket

  Serial1.println(F("AT+CSQ"));       // Check signal quality
  simread();                          // read response and print out
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CFUN=0"));    // Set phone functionality to minimal 
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CGDCONT=1,\"IP\",\"iot.1nce.net\""));   // Define PDP contect: PDP contect identifier, Set to IP, APN 
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CFUN=1"));    // Set phone funtionality to full
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+COPS?"));     // Check operator. See operator e.g Telia and if connected to NB or CAT-M
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CGNAPN"));    // Check if connected to the correct APN
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CGATT?"));    // Check if connected to GPRS service
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CNACT=0,1")); // Activate network
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CNACT?"));    // Check if network is active to adress
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CAOPEN=0,0,\"UDP\",\"164.92.164.168\",8080"));   // Open UDP connection/socket
  simread();

  LED_status();

  clrsimBuffer();

}

short* tempConvert(short* temp) {    // Scale teperature so the server can read it
  lengthCounter = 0;
  short t = 0;
  for (int i = 0; i < 1; i++) {               // Takes temp e.g 254 = 25,4 C and plus 300 to scale it up
    t = temp[i] + 300;
    temp[i] = t;
    lengthCounter++;
  }
  return temp;
}

void Sim_send(short* data, int len) {             // Create message and send to the server

  char msg[50];                          // To hold message
  char dat[10];                          // To hold incomming data
  char crcData[10];                      // To hold CRC
  char sendLength[20] = "AT+CASEND=0,";  // To hold at command that sends size of data to be send
  char msgLength[20];                    // Hold size of message

  short crc = 0;

  // Reset arrays
  memset(msg, 0, sizeof msg);           
  memset(dat, 0, sizeof dat);
  memset(crcData, 0, sizeof crcData);

  // Add to message in correct order
  sprintf(msg, IMEI);
  strcat(msg, model);
  strcat(msg, firmware);
  strcat(msg, extPower);
  for (int i = 0; i < len; i++) {
    sprintf(dat, "%d", data[i]);
    strcat(msg, dat);
  }
  strcat(msg, "30");

  crc = CRC16_xmodem(msg, strlen(msg));             // Find CRC from message

  sprintf(crcData, "%u", crc);

  // Add CRC to end of message
  for (int i = 0; i < (5 - strlen(crcData)); i++) {
    strcat(msg, "0");
  }
  strcat(msg, crcData);

  Serial.write(msg, strlen(msg));
  Serial.println();

  sprintf(msgLength, "%u", strlen(msg));          // Add message size to end of AT command
  strcat(sendLength, msgLength);

  Serial1.write(sendLength, strlen(sendLength));  // Send AT command
  Serial1.println();                              // Need ln when writing to sim module
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.write(msg, strlen(msg));  // Send message to server
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CAACK=0")); // Query Send Data Information
  simread();
  LED_status();
  clrsimBuffer();

  Serial1.println(F("AT+CARECV=0,50"));   // Read recieved message from server
  simread();
  LED_status();

  simReadData();    // Read message into array
}

void simReadData() {  // Read recieved message from server into array

  lengthCounter = 0;
  int start = 0;

  if (simBuffer[27] < 48) {               // Reads the number of bytes recieved from the server and put it into lengthCounter
    lengthCounter = simBuffer[26] - 48;
    start = 28;
  } else {
    lengthCounter = ((simBuffer[26] - 48) * 10) + (simBuffer[27] - 48);
    start = 29;
  }

  for (int i = 0; i < lengthCounter; i++) {     // Reads the message into simMsg array to be used
    simMsg[i] = simBuffer[i + start] - 48;
  }

  for (int i = 0; i < lengthCounter; i++)   // Print it to terminal
    Serial.print(simMsg[i]);

  Serial.println();

  clrsimBuffer();
}

void Sim_close() {                       // Used to close socket and APP network
  Serial1.println(F("AT+CACLOSE=0"));
  simread();

  Serial1.println(F("AT+CNACT=0,0"));
  simread();
}

byte* modbusValue() {             // Takes the recieved message from modbus slave and extract the data the correct way

  memset(value, 0, sizeof value);
  slaveReg = 0;

  byte state = modBuffer[1];
  byte j;

  lengthCounter = 0;

  switch (state) {
    case FC_readcoils:    // If state is FC_readcoils: split 1 bit data into 1 byte pr. bit.

      for (int i = 0; i < modBuffer[2]; i++) {    // Loop for number of bytes recieved

        j = modBuffer[3 + i];   // Put data byte into j

        for (int i = 0; i < modBuffer[2] * 8; i++) {    // Take on bit at a time and put into array of bytes
          value[i] = (j & (1 << i)) >> i;
          lengthCounter++;
        }
      }

      modbusPrint(modBuffer[2] * 8);    // Print out value

      slaveReg = 1;   // Slave registered

      break;

    case FC_readdiscreteinputs:   // Split 1 bit data into 1 byte pr. bit. Same as above      

      for (int i = 0; i < modBuffer[2]; i++) {

        j = modBuffer[3 + i];

        for (int i = 0; i < modBuffer[2] * 8; i++) {
          value[i] = (j & (1 << i)) >> i;
          lengthCounter++;
        }
      }

      modbusPrint(modBuffer[2] * 8);

      slaveReg = 1;

      break;

    case FC_readholdingregisters:   // Put data directly into value array, here data is 2 bytes long pr. adress

      for (int i = 0; i < modBuffer[2]; i++) {    // Loop for number af bytes

        value[i] = modBuffer[3 + i];    // Add data to array
        lengthCounter++;
      }

      modbusPrint(modBuffer[2]);

      slaveReg = 1;

      break;

    case FC_readinputregisters:   // Put data directly into value array, here data is 2 bytes long pr. adress. Same as above

      for (int i = 0; i < modBuffer[2]; i++) {

        value[i] = modBuffer[3 + i];
        lengthCounter++;
      }

      modbusPrint(modBuffer[2]);

      slaveReg = 1;

      break;

    // If write functions: give back value 1 for confirmation. Is not normally used.
    case FC_writesinglecoil:    

      value[0] = 1;
      modbusPrint(1);

      slaveReg = 1;

      break;

    case FC_writesingleregister:

      value[0] = 1;
      modbusPrint(1);

      slaveReg = 1;

      break;

    case FC_writecoils:

      value[0] = 1;
      modbusPrint(1);

      slaveReg = 1;

      break;

    case FC_writemultipleregisters:

      value[0] = 1;
      modbusPrint(1);

      slaveReg = 1;

      break;

    default:

      slaveReg = 0;

      break;
  }


  clrmodBuffer();
  return value;
}

void modbusPrint(int s) {         // Print out read value from modbus slaves
  for (int i = 0; i < s; i++) {
    Serial.print(value[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

bool comcheck(int dly, char* s) {           // Check if read communication is correct 

  bool check = 0;   // Bool for check 

  delay(dly);       // Custom delay depending on AT command

  while (!check) {  // Stay in while until correct response is read

    while (Serial1.available()) {           // Check response
      int numBytes = Serial1.available();
      for (int i = 0; i < numBytes; i++) {
        simBuffer[i] = Serial1.read();
      }
    }
    // Serial.write(simBuffer);

    if (strstr(simBuffer, s)) {       // If response is correct, set check and print out detected
      check = 1;
      Serial.print(F("Detected: "));
      Serial.println(s);
    } else {                          // Else check is not set
      check = 0;
      // Serial.println(F("Not detected"));
    }
  }
  clrsimBuffer();
  return check;
}

void simpow() {           // Used to power on or off sim module

  digitalWrite(43, LOW);
  delay(2000);
  digitalWrite(43, HIGH);
}

bool ATcomcheck(int dly) {                  // Same as comcheck, but only for check "OK" response and with no while

  bool check = 0;

  delay(dly);

  if (Serial1.available()) {
    int numBytes = Serial1.available();
    for (int i = 0; i < numBytes; i++) {
      simBuffer[i] = Serial1.read();
    }
  }
  // Serial.write(Buffer);
  if (strstr(simBuffer, "OK")) {
    check = 1;
    Serial.print(F("Detected: "));
    Serial.println("OK");
  } else {
    check = 0;
    // Serial.println(F("Not detected"));
  }

  clrsimBuffer();
  return check;
}

void siminit() {                    // Initiate sim module by checking if it can recieve "AT"

  while (stop) {                    // Stay in loop while stop is set
    for (int i = 0; i < 4; i++) {   // Check four times for OK response
      Serial1.println(F("AT"));     // Print AT
      start = ATcomcheck(300);      // Check for OK
      if (start) {                  // If start is set, stop while
        stop = 0;
      }
    }
    if (!start) {                   // If start is not set sim module is turned of, therefore turn on
      simpow();
    }
  }
  stop = 1;                         // Set stop for next siminit call

  clrsimBuffer();

}

void simread() {    // Read response after sending AT command

  delay(3000);      // Wait for sim module to respons correctly

  while (Serial1.available()) {           // While data incomming: Read into buffer
    int numBytes = Serial1.available();
    for (int i = 0; i < numBytes; i++) {
      simBuffer[i] = Serial1.read();
    }
  }
  // Serial.write(simBuffer);    // Write to terminal
}

void clrsimBuffer() {   // Clear simBuffer
  memset(simBuffer, 0, sizeof simBuffer);
}

void writeTerminal() {    // Function used to normally write to sim module via terminal

  if (Serial.available())
    Serial1.write(Serial.read());

  if (Serial1.available())
    Serial.write(Serial1.read());
}

void readcoils(byte slave, short adress, short qty) {  // Reads coils (if relays are on or off)

  //Define ADU size
  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  //Assempling ADU message
  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x01;  //Function code
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);

  //Calculate CRC16 for MODBUS
  CRC = CRC16_modbus(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  //Write AUD to MODBUS slave
  Serial2.write(ADU, ADUsize);

  //Read message from slave
  modbusread();
}

void readdiscreteinputs(byte slave, short adress, short qty) {  // Reads discreate inputs

  //Define ADU size
  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  //Assempling ADU message
  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x02;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);

  //Calculate CRC16 for MODBUS
  CRC = CRC16_modbus(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  //Write AUD to MODBUS slave
  Serial2.write(ADU, ADUsize);

  //Read message from slave
  modbusread();
}

void readholdingregisters(byte slave, short adress, short qty) {  // Read holding registers. Read function above for details on next few

  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x03;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);

  CRC = CRC16_modbus(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);


  Serial2.write(ADU, ADUsize);
  
  modbusread();
}

void readinputregisters(byte slave, short adress, short qty) {  // Read input registers

  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x04;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);

  CRC = CRC16_modbus(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial2.write(ADU, ADUsize);

  modbusread();
}

void writesinglecoil(byte slave, short adress, bool out) {  // Write single coil

  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x05;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);

  // 0xFF00 for turn on, and 0x0000 for turn off
  if (out > 0) {
    ADU[ADUsize++] = 0xFF;
    ADU[ADUsize++] = 0x00;
  } else {
    ADU[ADUsize++] = 0x00;
    ADU[ADUsize++] = 0x00;
  }

  CRC = CRC16_modbus(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial2.write(ADU, ADUsize);

  modbusread();
}

void writesingleregister(byte slave, short adress, short out) { // Write single register

  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x06;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(out);
  ADU[ADUsize++] = lowByte(out);

  CRC = CRC16_modbus(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial2.write(ADU, ADUsize);

  modbusread();
}

void writecoils(byte slave, short adress, short qty, byte* out) { // Write multiple coils

  byte ADU[50];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x0F;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);
  ADU[ADUsize++] = (qty / 8) + ((qty % 8) != 0);            // Calculate number of bytes pr. bit qty e.g 2 = 1 byte, 8 = 1 byte, 11 = 2 bytes.

  for (int i = 0; i < (qty / 8) + ((qty % 8) != 0); i++) {  // Put output array in bytes into the ADU
    ADU[ADUsize++] = out[i];
  }

  CRC = CRC16_modbus(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial2.write(ADU, ADUsize);

  modbusread();
}

void writemultipleregisters(byte slave, short adress, short qty, short* out) {  // Write multiple registers

  byte ADU[50];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x10;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);
  ADU[ADUsize++] = qty * 2;   // Two bytes pr. register

  for (int i = 0; i < qty; i++) {       // Put qty of data into ADU
    ADU[ADUsize++] = highByte(out[i]);
    ADU[ADUsize++] = lowByte(out[i]);
  }

  CRC = CRC16_modbus(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial2.write(ADU, ADUsize);

  modbusread();
}

void modbusread() {   // Read modbus response

  delay(300);   // Wait for slaves to process message

  while (Serial2.available()) {   // Stay in while while data available

    int numBytes = Serial2.available();   // Find number of bytes to read
    for (int i = 0; i < numBytes; i++) {
      modBuffer[i] = Serial2.read();      // Read into modBuffer
      Serial.print(modBuffer[i], HEX);      // Print to terminal
      Serial.print(" ");
    }
    Serial.println();
  }
}

void clrmodBuffer() {   // Clear modBuffer
  memset(modBuffer, 0, sizeof modBuffer);
}

unsigned short CRC16_modbus(byte* buf, int len) {    //Find modbus CRC16
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

unsigned short CRC16_xmodem(char* buf, int len) {    //Find xmodem CRC16
  int crc;
  char i;
  crc = 0;
  while (--len >= 0) {
    crc = crc ^ (int)*buf++ << 8;
    i = 8;
    do {
      if (crc & 0x8000)
        crc = crc << 1 ^ 0x1021;
      else
        crc = crc << 1;
    } while (--i);
  }
  return (crc);
}

void LED_status(void){

    if (strstr(simBuffer, "ERROR")) {
    digitalWrite(RED_LED, HIGH);
    delay(125);
    digitalWrite(RED_LED, LOW);
    delay(125);
    digitalWrite(RED_LED, HIGH);
    delay(125);
    digitalWrite(RED_LED, LOW);
    delay(125);
  }
    else if (strstr(simBuffer, "OK")) {
    digitalWrite(BLUE_LED, HIGH);
    delay(125);
    digitalWrite(BLUE_LED, LOW);
    delay(125);
    digitalWrite(BLUE_LED, HIGH);
    delay(125);
    digitalWrite(BLUE_LED, LOW);
    delay(125);
  }
  else{
    digitalWrite(BLUE_LED, HIGH);
    delay(250);
    digitalWrite(BLUE_LED, LOW);
    delay(250);
    digitalWrite(BLUE_LED, HIGH);
    delay(250);
    digitalWrite(BLUE_LED, LOW);
    delay(250);
  }
}

void response_check(char* s){

  if (strstr(simBuffer, s)) {
    digitalWrite(GREEN_LED, HIGH);
    delay(125);
    digitalWrite(GREEN_LED, LOW);
    delay(125);
    digitalWrite(GREEN_LED, HIGH);
    delay(125);
    digitalWrite(GREEN_LED, LOW);
    delay(125);
  }
}
