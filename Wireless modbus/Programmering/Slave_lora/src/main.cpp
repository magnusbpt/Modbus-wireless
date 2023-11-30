#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Wire.h>

// default "Wire" object: SDA = GP4, SCL = GP5, I2C0 peripheral
// our new wire object:
#define WIRE1_SDA 6 // Use GP2 as I2C1 SDA
#define WIRE1_SCL 7 // Use GP3 as I2C1 SCL
arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);

#define loraSerial Serial1

#define i2c_ext Wire
#define i2c_int Wire1

#define pirSensor 4
#define AIR_INT 5
#define AIR_RES 6
#define LoRa_RES 7
#define LoRa_BOOT 8
#define Digital_in 9
#define Vib_Sensor 10
#define ADC1_LUX 27
#define ADC2_DB 28

#define dataCommand 0x2C0D
#define temp_write 0x44
#define temp_read 0x45
#define GPIO_write 0x68
#define GPIO_read 0x69
#define CO2_write 0x32
#define CO2_read 0x33

#define tempAdress 0x0000
#define humidityAdress 0x0001
#define luxAdress 0x0002
#define dbAdress 0x0003
#define CO2Adress 0x0004

#define digitalAdress 0x0005
#define pirAdress 0x0006
#define vibAdress 0x0007

#define I2C_extAdress 0x0008

char readBuffer[200];
char Buf[30] = "AT+TEST=TXLRPKT,\"";
char length[20];
char messageBuffer[50];

int messageHead = 1;
int i = 0;
int pause = 0;
int interval = 0;
int delta = 0;
char ID = 0;
unsigned long time_old = 0;

short luxValue = 0;
short dbValue = 0;
short temp = 0;
short humidity = 0;
short CO2 = 0;
char digitalValue = 0;
char pirValue = 0;
char vibValue = 0;
short temp_last = 0;
short humidity_last = 0;
short lux_last = 0;
short CO2_last = 0;
short db_last = 0;

bool pirFlag = true;
bool msgFlag = false;

/***************Define functions***************/
void loraRead();
void millisDelay(int);
void slaveInit();
void readSensor();
unsigned long minutes();
int powOf(int, int);
unsigned short CRC16_modbus(char *, int);
void loraFlush();

void setup()
{
  i2c_int.begin();
  i2c_ext.begin();
  Serial.begin(9600);
  loraSerial.begin(9600);
  analogReadResolution(12);

  pinMode(pirSensor, INPUT);

  delay(7000);

  slaveInit();

  readSensor();

  humidity_last = humidity;
  lux_last = luxValue;
  CO2_last = CO2;
  db_last = dbValue;

  msgFlag = true;

  loraSerial.println("AT+LOWPOWER");
  loraRead();
  memset(readBuffer, 0, sizeof readBuffer);
}

void loop()
{
  delay(1000);

  readSensor();

  int tempDiff = (temp_last - temp) / temp;
  int humidityDiff = (humidity_last - humidity) / humidity;
  int luxDiff = (lux_last - luxValue) / luxValue;

  if (delta > 0)
  {
    if (tempDiff > delta || tempDiff < -delta)
    {
      msgFlag = true;
      temp_last = temp;
      messageBuffer[messageHead++] = highByte(tempAdress);
      messageBuffer[messageHead++] = lowByte(tempAdress);
      messageBuffer[messageHead++] = highByte(temp);
      messageBuffer[messageHead++] = lowByte(temp);
    }
    if (humidityDiff > delta || humidityDiff < -delta)
    {
      msgFlag = true;
      humidity_last = humidity;
      messageBuffer[messageHead++] = highByte(humidityAdress);
      messageBuffer[messageHead++] = lowByte(humidityAdress);
      messageBuffer[messageHead++] = highByte(humidity);
      messageBuffer[messageHead++] = lowByte(humidity);
    }
    if (luxDiff > delta || luxDiff < -delta)
    {
      msgFlag = true;
      lux_last = luxValue;
      messageBuffer[messageHead++] = highByte(luxAdress);
      messageBuffer[messageHead++] = lowByte(luxAdress);
      messageBuffer[messageHead++] = highByte(luxValue);
      messageBuffer[messageHead++] = lowByte(luxValue);
    }
  }

  if ((minutes() - time_old > interval) || msgFlag)
  {
    msgFlag = false;
    time_old = minutes();

    messageBuffer[messageHead++] = highByte(tempAdress);
    messageBuffer[messageHead++] = lowByte(tempAdress);
    messageBuffer[messageHead++] = highByte(temp);
    messageBuffer[messageHead++] = lowByte(temp);

    messageBuffer[messageHead++] = highByte(humidityAdress);
    messageBuffer[messageHead++] = lowByte(humidityAdress);
    messageBuffer[messageHead++] = highByte(humidity);
    messageBuffer[messageHead++] = lowByte(humidity);

    messageBuffer[messageHead++] = highByte(luxAdress);
    messageBuffer[messageHead++] = lowByte(luxAdress);
    messageBuffer[messageHead++] = highByte(luxValue);
    messageBuffer[messageHead++] = lowByte(luxValue);

    messageBuffer[messageHead++] = highByte(dbAdress);
    messageBuffer[messageHead++] = lowByte(dbAdress);
    messageBuffer[messageHead++] = highByte(dbValue);
    messageBuffer[messageHead++] = lowByte(dbValue);

    messageBuffer[messageHead++] = highByte(CO2Adress);
    messageBuffer[messageHead++] = lowByte(CO2Adress);
    messageBuffer[messageHead++] = highByte(CO2);
    messageBuffer[messageHead++] = lowByte(CO2);

    messageBuffer[messageHead++] = highByte(digitalAdress);
    messageBuffer[messageHead++] = lowByte(digitalAdress);
    messageBuffer[messageHead++] = 0x00;
    messageBuffer[messageHead++] = digitalValue;

    messageBuffer[messageHead++] = highByte(pirAdress);
    messageBuffer[messageHead++] = lowByte(pirAdress);
    messageBuffer[messageHead++] = 0x00;
    messageBuffer[messageHead++] = pirValue;

    messageBuffer[messageHead++] = highByte(vibAdress);
    messageBuffer[messageHead++] = lowByte(vibAdress);
    messageBuffer[messageHead++] = 0x00;
    messageBuffer[messageHead++] = vibValue;

    messageBuffer[0] = ID;

    short CRC = 0; // To hold CRC

    CRC = CRC16_modbus(messageBuffer, messageHead); // Calculate CRC

    messageBuffer[messageHead++] = highByte(CRC);
    messageBuffer[messageHead++] = lowByte(CRC);

    char loraTX[50] = "AT+TEST=TXLRPKT, \"";

    loraSerial.println("ON");

    millisDelay(100);

    loraSerial.println("AT+MODE=TEST");
    loraFlush();

    millisDelay(100);

    loraSerial.println("AT+TEST=RFCFG,868,SF12,125,8,8,22,ON,OFF,OFF");
    loraFlush();

    millisDelay(100);

    /**********Test***********/

    ID = 0x01;

    char message1[50] = {0x01, 0x00, 0x00, 0x00, 0xFF, 0x80, 0x59};

    /************************/
    char temp1[1];
    char temp2 = 0;

    for(int i = 0; i < 7; i++)
    {
      temp2 = message1[i] >> 4;
      sprintf(temp1, "%X", temp2);
      loraTX[(i*2) + 18] = temp1[0];

      temp2 = message1[i] & 0x0F;
      sprintf(temp1, "%X", temp2);
      loraTX[(i*2) + 19] = temp1[0];
    }

    memset(messageBuffer, 0, sizeof messageBuffer);

    strcat(loraTX, "\"\r\n");

    loraSerial.write(loraTX, strlen(loraTX));
    millisDelay(800);
    loraRead();
    memset(readBuffer, 0, sizeof readBuffer);

    millisDelay(100);

    loraSerial.println("AT+TEST=RXLRPKT");
    loraRead();
    memset(readBuffer, 0, sizeof readBuffer);

    millisDelay(100);

    bool flag = false;
    bool flag2 = false;
    while (!flag)
    {
      unsigned long time_now = millis();
      while (!loraSerial.available())
      {
        if (millis() - time_now > 10000)
        {
          Serial.println("No message received");
          flag2 = true;
          break;
        }
      }

      if (flag2)
      {
        loraSerial.write(loraTX, strlen(loraTX));
        millisDelay(800);
        loraRead();
        memset(readBuffer, 0, sizeof readBuffer);

        millisDelay(200);

        loraSerial.println("AT+TEST=RXLRPKT");
        loraRead();
        memset(readBuffer, 0, sizeof readBuffer);

        flag2 = false;
      }
      else
      {
        loraRead();
        if (strchr(readBuffer, ID) > 0)
        {
          Serial.println("Message received");
          flag = true;
        }
      }

      memset(readBuffer, 0, sizeof readBuffer);
    }


    loraSerial.println(F("AT+LOWPOWER"));
    loraFlush();

    messageHead = 1;
  }

}

void loraRead()
{ // Read response after sending AT command

  millisDelay(200); // Wait for sim module to respons correctly
  i = 0;

  while (loraSerial.available())
  { // While data incomming: Read into buffer
    readBuffer[i] = loraSerial.read();
    i++;
  }
  Serial.println(readBuffer); // Write to terminal
}

void loraFlush()
{ // Read response after sending AT command

  millisDelay(100); // Wait for sim module to respons correctly
  i = 0;
  char t = 0;

  while (loraSerial.available())
  { // While data incomming: Read into buffer
    t = loraSerial.read();
    i++;
  }
}

void slaveInit()
{
  Serial.println("Slave init");
  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x1D);                   // sends five bytes
  i2c_int.write(0x00);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  millisDelay(100);

  
  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x1E);                   // sends five bytes
  i2c_int.write(0x00);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  millisDelay(100);
  
  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x1F);                   // sends five bytes
  i2c_int.write(0x00);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  millisDelay(100);
  
  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x23);                   // sends five bytes
  i2c_int.write(0x00);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x24);                   // sends five bytes
  i2c_int.write(0x00);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x25);                   // sends five bytes
  i2c_int.write(0x00);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x14);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  // Wire.requestFrom(GPIO_read, 1);
  char delta_pause = Wire.read();

  delta = (delta_pause >> 3) / 100;

  pause = delta_pause & 0x0F;

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x16);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  millisDelay(100);

  // Wire.requestFrom(GPIO_read, 1);
  short ID_int = Wire.read();

  ID = ID_int << 4;

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_write); // transmit to device #0x68
  i2c_int.write(0x15);                   // sends five bytes
  i2c_int.endTransmission();             // stop transmitting

  millisDelay(100);

  // Wire.requestFrom(GPIO_read, 1);
  ID_int = Wire.read();

  ID = ID | (ID_int >> 4);

  interval = ID_int & 0x0F;

  if (interval == 0x01)
  {
    interval = 30;
  }
  else if (interval == 0x02)
  {
    interval = 60;
  }
  else if (interval == 0x04)
  {
    interval = 720;
  }
  else if (interval == 0x08)
  {
    interval = 1440;
  }
  
}

void millisDelay(int delayTime)
{

  unsigned long time_now = millis();

  while (millis() - time_now < delayTime)
  {
    // wait.
  }
}

unsigned long minutes()
{
  return (millis() / 60000);
}

void readSensor()
{

  Serial.println("Read sensor");

  /******Lux sensor******/
  luxValue = analogRead(ADC1_LUX);

  /******dB sensor******/
  dbValue = analogRead(ADC2_DB);

  /******Pir sensor******/
  static int pirTime = 0;
  if ((millis() - pirTime > pause * 60000) || pirFlag)
  {
    pirValue = digitalRead(pirSensor);
    pirTime = millis();
    pirFlag = false;
  }

  /******Temp and humidity******/
  i2c_int.beginTransmission(temp_write); // transmit to device #0x44
  i2c_int.write(highByte((short)dataCommand));
  i2c_int.write(lowByte((short)dataCommand));
  i2c_int.endTransmission(); // stop transmitting

  millisDelay(100);

  // Wire.requestFrom(temp_read, 6);
  char tempHigh = Wire.read();
  char tempLow = Wire.read();
  int check1 = Wire.read(); // Do not care. Just a place holder
  char humHigh = Wire.read();
  char humLow = Wire.read();
  int check2 = Wire.read();
  temp = (tempHigh << 8) | (tempLow << 0);
  humidity = (humHigh << 8) | (humLow << 0);

  temp = -45 + (175 * (temp / (powOf(2, 16) - 1)));
  humidity = 100 * (humidity / (powOf(2, 16) - 1));

  /******Vibration******/
  int vibCounter = 0;
  for (int i = 0; i < 20; i++)
  {
    vibValue = digitalRead(Vib_Sensor);
    if (vibValue == 1)
    {
      vibCounter++;
    }
  }

  if (vibCounter > 10)
  {
    vibValue = 1;
  }
  else
  {
    vibValue = 0;
  }

  /******Digital in******/
  digitalValue = digitalRead(Digital_in);

  /******CO2 sensor******/
}

int powOf(int base, int exp)
{
  int result = 1;
  for (int i = 0; i < exp; i++)
  {
    result = result * base;
  }
  return result;
}

unsigned short CRC16_modbus(char *buf, int len)
{ // Find modbus CRC16
  unsigned int crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (unsigned int)buf[pos]; // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--)
    { // Loop over each bit
      if ((crc & 0x0001) != 0)
      {            // If the LSB is set
        crc >>= 1; // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else         // Else LSB is not set
        crc >>= 1; // Just shift right
    }
  }

  return crc;
}
