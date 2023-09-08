#include <ModbusMaster.h>
#include <AltSoftSerial.h>
#include <SoftwareSerial.h>


// instantiate ModbusMaster object
ModbusMaster node;

AltSoftSerial altSerial;

SoftwareSerial mySerial(8, 9);  // RX, TX

byte Buffer[50];

void setup() {
  // use Serial (port 0); initialize Modbus communication baud rate
  Serial.begin(9600);
  altSerial.begin(9600);

  // communicate with Modbus slave ID 2 over Serial (port 0)
  node.begin(1, altSerial);
}


void loop() {
  delay(2000);

  static uint32_t i;
  uint8_t j, result;
  uint16_t data[6];

  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  // node.setTransmitBuffer(0, lowWord(i));

  // // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  // node.setTransmitBuffer(1, highWord(i));

  // slave: write TX buffer to (2) 16-bit registers starting at register 0
  // result = node.writeMultipleRegisters(0, 2);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  node.readHoldingRegisters(4, 2);

  // do something with data if read is successful
  if (node.available()) {
    for (j = 0; j < 6; j++) {
      data[j] = node.getResponseBuffer(j);
      Serial.println(data[j]);
    }
  }
  // delay(1000);

  // node.setTransmitBuffer(0, 1);
  // node.writeMultipleCoils(0, 2);
  // readmsg();
  // node.readCoils(0, 2);
  // readmsg();
}

// void readmsg(){

//  while (node.available() > 0) {

//   int numBytes = node.available();
//   for (int i = 0; i < numBytes; i++) {
//     Buffer[i] = node.receive();
//     Serial.print(Buffer[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();
// }
//   node.clearResponseBuffer();
//   memset(Buffer, 0, sizeof Buffer);
// }
