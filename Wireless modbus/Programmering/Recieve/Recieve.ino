
unsigned char readBuffer[200];
int i=0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  delay(2000);

  Serial1.println(F("AT+MODE=TEST"));
  read();

  delay(300);

  Serial1.println(F("AT+TEST=RXLRPKT"));
  read();
}

void loop() {
  
  while (Serial.available() > 0) {
    Serial1.write(Serial.read());
  }

  while (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }
}

void read() {    // Read response after sending AT command

  delay(100);      // Wait for sim module to respons correctly

  while (Serial1.available()) {           // While data incomming: Read into buffer

    int numBytes = Serial1.available();
    for (int i = 0; i < numBytes; i++) {
      readBuffer[i] = Serial1.read();
    }
  }
  Serial.write((char*)readBuffer);    // Write to terminal
  Serial.println();
}
