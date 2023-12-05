
unsigned char readBuffer[200];

char Buf[30] = "AT+TEST=TXLRPKT,\"";

char length[20];

int k = 0;

int i = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  delay(2000);

  Serial1.println(F("AT+MODE=TEST"));
  read();
  memset(readBuffer, 0, sizeof readBuffer);

  delay(300);

  Serial1.println(F("AT+TEST=RFCFG,868,SF12,125,8,8,22,ON,OFF,OFF"));
  read();  
  memset(readBuffer, 0, sizeof readBuffer);

}

void loop() {

  delay(1000); 

  Serial1.println("AT+TEST=TXLRPKT, \"AA\"");                              // Need ln when writing to sim module
  read();
  memset(readBuffer, 0, sizeof readBuffer);

  delay(300);

  Serial1.println(F("AT+TEST=RXLRPKT"));
  read();
  memset(readBuffer, 0, sizeof readBuffer);

  // while (!Serial1.available()){}
  // read();
  // memset(readBuffer, 0, sizeof readBuffer);


}

void read() {    // Read response after sending AT command

  delay(100);      // Wait for sim module to respons correctly
  i = 0;

  while (Serial1.available()) {           // While data incomming: Read into buffer
      readBuffer[i] = Serial1.read();
      i++;
  }
  Serial.write((char*)readBuffer);    // Write to terminal
  Serial.println();
}
