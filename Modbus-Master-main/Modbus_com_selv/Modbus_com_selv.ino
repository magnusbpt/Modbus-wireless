#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <math.h>

//AltSoftSerial Serial1;
SoftwareSerial mySerial(8, 9);  // RX, TX

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);

  

  // writesinglecoil(1, 1, 0);

  // readcoils(3, 0, 1);

  // writecoils(1, 0, 1, 0);

  // readdiscreteinputs(1, 0, 1);

  // readinputregisters(1, 0, 1);

  // writecoils(3, 0, 1, 1);

  // writesingleregister(1, 0, 1);

  // byte Amsg[] = {0x01, 0x08, 0x06, 0x00, 0xAB, 0x5F, 0x77, 0x01, 0x01, 0x27, 0x40};
  // Serial1.write(Amsg, sizeof(Amsg));
  // readmsg();
   
  // byte Amsg[] = {0x01, 0x48, 0x0C, 0x00, 0xF7, 0xA6, 0x71, 0x01, 0x01, 0x00, 0xF7, 0xA6, 0x72, 0x02, 0x01, 0x29, 0x69};
  // Serial1.write(Amsg, sizeof(Amsg));
  // readmsg();
}

void loop() {
  writesinglecoil(3, 0, 1);
  // delay(300);
  // readholdingregisters(1, 1, 6);
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
  CRC = CRC16_2(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  //Write AUD to MODBUS slave
  Serial1.write(ADU, ADUsize);

  //Read message from slave
  readmsg();
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
  CRC = CRC16_2(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  //Write AUD to MODBUS slave
  Serial1.write(ADU, ADUsize);

  //Read message from slave
  readmsg();
}

void readholdingregisters(byte slave, short adress, short qty) {

  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x43;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);

  CRC = CRC16_2(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial1.write(ADU, ADUsize);

  readmsg();
}

void readinputregisters(byte slave, short adress, short qty) {

  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x04;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);

  CRC = CRC16_2(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial1.write(ADU, ADUsize);

  readmsg();
}

void writesinglecoil(byte slave, short adress, bool out) {

  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x05;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);

  if (out > 0) {
    ADU[ADUsize++] = 0xFF;
    ADU[ADUsize++] = 0x00;
  } else {
    ADU[ADUsize++] = 0x00;
    ADU[ADUsize++] = 0x00;
  }

  CRC = CRC16_2(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial1.write(ADU, ADUsize);

  readmsg();
}

void writesingleregister(byte slave, short adress, short out) {

  byte ADU[8];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x06;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(out);
  ADU[ADUsize++] = lowByte(out);

  CRC = CRC16_2(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial1.write(ADU, ADUsize);

  readmsg();
}

void writecoils(byte slave, short adress, short qty, short out) {

  byte ADU[11];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x0F;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);
  ADU[ADUsize++] = (qty / 8) + ((qty % 8) != 0);
  if (out > 0xFF) {
    ADU[ADUsize++] = lowByte(out);
    ADU[ADUsize++] = highByte(out);
  } else {
    ADU[ADUsize++] = lowByte(out);
  }

  CRC = CRC16_2(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial1.write(ADU, ADUsize);

  readmsg();
}

void writemultipleregisters(byte slave, short adress, short qty, short out1, short out2) {

  byte ADU[13];
  byte ADUsize = 0;
  int CRC;

  ADU[ADUsize++] = slave;
  ADU[ADUsize++] = 0x10;
  ADU[ADUsize++] = highByte(adress);
  ADU[ADUsize++] = lowByte(adress);
  ADU[ADUsize++] = highByte(qty);
  ADU[ADUsize++] = lowByte(qty);
  ADU[ADUsize++] = qty * 2;
  ADU[ADUsize++] = highByte(out1);
  ADU[ADUsize++] = lowByte(out1);
  ADU[ADUsize++] = highByte(out2);
  ADU[ADUsize++] = lowByte(out2);

  CRC = CRC16_2(ADU, ADUsize);
  ADU[ADUsize++] = lowByte(CRC);
  ADU[ADUsize++] = highByte(CRC);

  Serial1.write(ADU, ADUsize);

  readmsg();
}


unsigned int CRC16_2(unsigned char *buf, int len) {
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

void readmsg() {
  byte Buffer[50];

  delay(700);

  while (Serial1.available()) {

    int numBytes = Serial1.available();
    for (int i = 0; i < numBytes; i++) {
      Buffer[i] = Serial1.read();
      Serial.print(Buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  memset(Buffer, 0, sizeof Buffer);
}
