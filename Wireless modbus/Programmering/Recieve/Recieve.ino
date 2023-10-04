
static char message[512];
int i = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  millisDelay(2000);

  Serial1.println(F("AT+MODE=TEST"));
  read();
  // memset(message, 0, sizeof message);

  millisDelay(300);

  Serial1.println(F("AT+TEST=RFCFG,868,SF12,125,8,8,22,ON,OFF,OFF"));
  read();
  // memset(message, 0, sizeof message);

}

void loop() {
  Serial1.println(F("AT+TEST=RXLRPKT"));
  read();
  // memset(message, 0, sizeof message);

  while (!Serial1.available()) {}

  millisDelay(300);

  Serial1.println("AT+TEST=TXLRPKT, \"BB\"");
  read();
    // memset(message, 0, sizeof message);

  // if (strstr(message, "AA")) {
    // memset(message, 0, sizeof message);
    // delay(3000);
    // Serial1.println("AT+TEST=TXLRPKT, \"BB\"");
    // read();
    // memset(message, 0, sizeof message);
  // }
}

void read() {  // Read response after sending AT command

  millisDelay(300);  // Wait for sim module to respons correctly

  while (Serial1.available()) {  // While data incomming: Read into buffer

  Serial.write(Serial1.read());
  delay(10);

  //   int numBytes = Serial1.available();
  //   for (int i = 0; i < numBytes; i++) {
  //     message[i] = Serial1.read();
  //   }
  // }
  // Serial.write((char*)message);  // Write to terminal
  // Serial.println();

}
}

void millisDelay(int delayTime) {

  unsigned long time_now = millis();

  while (millis() - time_now < delayTime) {
    //wait.
  }
}
