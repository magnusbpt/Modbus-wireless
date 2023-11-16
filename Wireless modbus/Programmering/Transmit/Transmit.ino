

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {

  // read from port 0, send to port 1:

  while (Serial.available()) {

    Serial1.write(Serial.read());

  }

  while (Serial1.available()) {

    Serial.write(Serial1.read());

  }

}