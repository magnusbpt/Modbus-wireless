
char message[300];

char length[20];

int k = 0;

int i = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  millisDelay(2000);

  Serial1.println(F("AT+MODE=TEST"));
  read();
  memset(message, 0, sizeof message);

  millisDelay(300);

  Serial1.println(F("AT+TEST=RFCFG,868,SF12,125,8,8,22,ON,OFF,OFF"));
  read();
  memset(message, 0, sizeof message);
}

void loop() {

  unsigned long timerStart = 0;
  unsigned long timerEnd = 0;
  

  millisDelay(300);

  Serial1.println("AT+TEST=TXLRPKT, \"AA\"");  // Need ln when writing to sim module
  read();
  memset(message, 0, sizeof message);

  millisDelay(300);

  Serial1.println(F("AT+TEST=RXLRPKT"));
  read();

  timerStart = millis();
  
  while (!Serial1.available() && !(((millis()) - timerStart) > 1000)) {}
  read();

  // read();
  // memset(message, 0, sizeof message);

  // Serial1.println(F("AT+TEST=RXLRPKT"));
  // read();
  // memset(message, 0, sizeof message);

  // while (!Serial1.available()) {}

  // read();
  // memset(message, 0, sizeof message);
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