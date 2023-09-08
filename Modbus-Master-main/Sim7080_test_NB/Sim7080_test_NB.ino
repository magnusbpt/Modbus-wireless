#include <TimeLib.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

SoftwareSerial simSerial(8, 9);  // RX, TX

char Buffer[200];

time_t t = now();

bool start = 0;
bool stop = 1;

int lognr = 1;
int teststart = 0;

void setup() {
  simSerial.begin(9600);
  Serial.begin(9600);

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  //---------------Set time----------------//
  setTime(16, 12, 0, 21, 2, 23);  //setTime(hours, minutes, seconds, days, months, years)

  delay(2000);

  siminit();
}

void loop() {

  if ((!(minute() % 10) && second() == 0) || start) {

    bool test = 0;
    bool test2 = 0;
    bool test3 = 0;

    siminit();

    printlog();
    while (!test && !test2 && !test3) {
      simSerial.println(F("AT+CEREG?"));
      test = comcheck(100, "+CEREG: 0,5");

      simSerial.println(F("AT+CEREG?"));
      test2 = comcheck(100, "+CEREG: 0,1");

      simSerial.println(F("AT+CEREG?"));
      test3 = comcheck(100, "+CEREG: 0,0");
    }
    printtime();

    if (test3) {
      simSerial.println(F("Error connecting: Restarting"));
      simpow();
    } else {
      Sim_Connect();
    }

    start = 0;
  }
  writeTerminal();
}

void writeTerminal() {

  if (Serial.available())
  simSerial.write(Serial.read());

  if (simSerial.available())
    Serial.write(simSerial.read());

}

void Sim_Connect() {

  simSerial.println(F("AT+CPIN?"));
  readbuffer();

  simSerial.println(F("AT+CSQ"));
  readbuffer();

  simSerial.println(F("AT+CFUN=0"));
  readbuffer();

  simSerial.println(F("AT+CGDCONT=1,\"IP\",\"iot.1nce.net\""));
  readbuffer();

  simSerial.println(F("AT+CFUN=1"));
  readbuffer();

  delay(3000);

  simSerial.println(F("AT+COPS?"));
  readbuffer();

  simSerial.println(F("AT+CGNAPN"));
  readbuffer();

  simSerial.println(F("AT+CGATT?"));
  readbuffer();

  simSerial.println(F("AT+CNACT=0,1"));
  readbuffer();

  simSerial.println(F("AT+CNACT?"));
  readbuffer();

  simSerial.println(F("AT+CAOPEN=0,0,\"UDP\",\"164.92.139.57\",8080"));
  readbuffer();

  simSerial.println(F("AT+CASEND=0,2"));
  readbuffer();

  simSerial.println(F("00"));
  readbuffer();

  simSerial.println(F("AT+CAACK=0"));
  readbuffer();

  simSerial.println(F("AT+CARECV=0,2"));
  readbuffer();

  simSerial.println(F("AT+CACLOSE=0"));
  readbuffer();

  simSerial.println(F("AT+CNACT=0,0"));
  readbuffer();

  simSerial.println(F("AT+CPOWD=1"));
  readbuffer();
}

void readbuffer() {
  delay(3000);

  if (simSerial.available()) {

    int numBytes = simSerial.available();
    for (int i = 0; i < numBytes; i++) {
      Buffer[i] = simSerial.read();
    }
  }
  Serial.write(Buffer);
  memset(Buffer, 0, sizeof Buffer);
}

void printlog() {
  delay(50);
  Serial.print("-----------------------------Log nr: ");
  Serial.print(lognr);
  Serial.print("  ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.print("  ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.println("-----------------------------");

  lognr++;
}

void printtime() {
  delay(50);
  Serial.print("-----------------------------time: ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.print("  ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.println("-----------------------------");
}

bool comcheck(int dly, char* s) {

  bool check = 0;

  delay(dly);

  if (simSerial.available()) {
    int numBytes = simSerial.available();
    for (int i = 0; i < numBytes; i++) {
      Buffer[i] = simSerial.read();
    }
  }
  // Serial.write(Buffer);
  if (strstr(Buffer, s)) {
    check = 1;
    Serial.print(F("Detected: "));
    Serial.println(s);
  } else {
    check = 0;
    // Serial.println(F("Not detected"));
  }

  memset(Buffer, 0, sizeof Buffer);
  return check;
}

void simpow() {

  digitalWrite(5, LOW);
  delay(2000);
  digitalWrite(5, HIGH);
}

void siminit() {

  while (stop) {
    for (int i = 0; i < 4; i++) {
      simSerial.println(F("AT"));
      start = comcheck(300, "OK");
      if (start) {
        stop = 0;
      }
    }
    if (!start) {
      simpow();
    }
  }
  stop = 1;
}
