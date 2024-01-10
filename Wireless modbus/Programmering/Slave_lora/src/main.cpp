#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Wire.h>

// our new wire object:
#define WIRE1_SDA 6 // Use GP6 as I2C1 SDA
#define WIRE1_SCL 7 // Use GP7 as I2C1 SCL
arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);

// Define serial port for LoRa
#define loraSerial Serial1

// Define I2C ports
#define i2c_ext Wire
#define i2c_int Wire1

// Define pins
#define Digital_in 2
#define AIR_RES 3
#define AIR_INT 4
#define pirSensor 10
#define Vib_Sensor 11
#define LED 14
#define LoRa_RES 18
#define LoRa_BOOT 19
#define FAN 20
#define ADC1_LUX 27
#define ADC2_DB 28

// Define I2C adresses
#define dataCommand 0x2C0D
#define temp_Adress 0x44
#define GPIO_Adress 0x68
#define CO2_Adress 0x32

// Define sensor adresses
#define tempPort 0x0001
#define humidityPort 0x0002
#define luxPort 0x0003
#define dbPort 0x0004
#define CO2Port 0x0005

#define digitalPort 0x0006
#define pirPort 0x0007
#define vibPort 0x0008

#define I2C_extPort 0x0009

// Define buffers
char readBuffer[200];
char Buf[30] = "AT+TEST=TXLRPKT,\"";
char length[20];
char messageBuffer[50];

// Define variables
int messageHead = 1;
int pause = 0;
int interval = 0;
int delta = 0;
char ID = 0;
unsigned long time_old = 0;

unsigned short luxValue = 0;
unsigned short dbValue = 0;
uint16_t temp = 0;
uint16_t humidity = 0;
unsigned short CO2 = 0;
unsigned char digitalValue = 0;
unsigned char pirValue = 0;
unsigned char vibValue = 0;
unsigned short temp_last = 0;
unsigned short humidity_last = 0;
unsigned short lux_last = 0;
unsigned short CO2_last = 0;
short db_last = 0;

bool pirFlag = false;
bool pirStart = true;
bool msgFlag = true;

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
  // Begin communication
  i2c_int.begin();
  i2c_ext.begin();
  Serial.begin(9600);
  loraSerial.begin(9600);

  // Change ADC resolution
  analogReadResolution(12);

  // Set pins
  pinMode(pirSensor, INPUT);
  pinMode(Digital_in, INPUT);
  pinMode(LoRa_RES, OUTPUT);
  pinMode(LoRa_BOOT, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LoRa_RES, HIGH);
  digitalWrite(LoRa_BOOT, HIGH);

  // Startup delay
  millisDelay(7000);

  // Initialize slave
  slaveInit();

  // Read sensors
  readSensor();

  // Set first last values for comparison
  humidity_last = humidity;
  lux_last = luxValue;
  CO2_last = CO2;
  db_last = dbValue;

  // Set LoRa module to sleep
  loraSerial.println("AT+LOWPOWER");
  loraFlush();
}

void loop()
{
  // Wait 1 second before reading sensors
  millisDelay(1000);

  // Read sensors
  readSensor();

  // Calculate difference between last and current value
  int tempDiff = ((temp_last - temp) / temp) * 100;
  int humidityDiff = ((humidity_last - humidity) / humidity) * 100;
  int luxDiff = ((lux_last - luxValue) / luxValue) * 100;
  int CO2Diff = ((CO2_last - CO2) / CO2) * 100;

  // If difference is bigger than delta, put data into message buffer
  if (delta > 0)
  {
    if ((tempDiff > delta) || (tempDiff < -delta)) // If temperature change is bigger than delta
    {
      msgFlag = true; //Set flag to make message
      temp_last = temp; //Set last data
    }
    if ((humidityDiff > delta) || (humidityDiff < -delta)) // If humidity change is bigger than delta
    {
      msgFlag = true; //Set flag to make message
      humidity_last = humidity; //Set last data
    }
    if ((luxDiff > delta) || (luxDiff < -delta)) // If lux change is bigger than delta
    {
      msgFlag = true; //Set flag to make message
      lux_last = luxValue; //Set last data
    }
    if ((CO2Diff > delta) || (CO2Diff < -delta)) // If CO2 change is bigger than delta
    {
      msgFlag = true; //Set flag to make message
      CO2_last = CO2; //Set last data
    }
  }

  if (((minutes() - time_old) > interval) || msgFlag)
  {
    msgFlag = false;
    time_old = minutes();

    messageBuffer[messageHead++] = highByte(tempPort);
    messageBuffer[messageHead++] = lowByte(tempPort);
    messageBuffer[messageHead++] = highByte(temp);
    messageBuffer[messageHead++] = lowByte(temp);
    messageBuffer[messageHead++] = 0x02;

    messageBuffer[messageHead++] = highByte(humidityPort);
    messageBuffer[messageHead++] = lowByte(humidityPort);
    messageBuffer[messageHead++] = highByte(humidity);
    messageBuffer[messageHead++] = lowByte(humidity);
    messageBuffer[messageHead++] = 0x02;

    messageBuffer[messageHead++] = highByte(luxPort);
    messageBuffer[messageHead++] = lowByte(luxPort);
    messageBuffer[messageHead++] = highByte(luxValue);
    messageBuffer[messageHead++] = lowByte(luxValue);
    messageBuffer[messageHead++] = 0x02;

    messageBuffer[messageHead++] = highByte(CO2Port);
    messageBuffer[messageHead++] = lowByte(CO2Port);
    messageBuffer[messageHead++] = highByte(CO2);
    messageBuffer[messageHead++] = lowByte(CO2);
    messageBuffer[messageHead++] = 0x02;

    messageBuffer[messageHead++] = highByte(digitalPort);
    messageBuffer[messageHead++] = lowByte(digitalPort);
    messageBuffer[messageHead++] = 0x00;
    messageBuffer[messageHead++] = digitalValue;
    messageBuffer[messageHead++] = 0x01;

    if (pirFlag)
    {
      messageBuffer[messageHead++] = highByte(pirPort);
      messageBuffer[messageHead++] = lowByte(pirPort);
      messageBuffer[messageHead++] = 0x00;
      messageBuffer[messageHead++] = pirValue;
      messageBuffer[messageHead++] = 0x01;
    }

    messageBuffer[messageHead++] = highByte(vibPort);
    messageBuffer[messageHead++] = lowByte(vibPort);
    messageBuffer[messageHead++] = 0x00;
    messageBuffer[messageHead++] = vibValue;
    messageBuffer[messageHead++] = 0x01;

    messageBuffer[0] = ID;

    short CRC = 0; // To hold CRC

    CRC = CRC16_modbus(messageBuffer, messageHead); // Calculate CRC

    messageBuffer[messageHead++] = highByte(CRC);
    messageBuffer[messageHead++] = lowByte(CRC);

   char loraTX[100] = "AT+TEST=TXLRPKT, \""; //Make array for LoRa message 

    loraSerial.println("ON"); //Wakeup LoRa module

    millisDelay(100);

    loraSerial.println("AT+MODE=TEST"); //Enter test mode
    loraFlush(); //Read and flush message

    millisDelay(100);

    loraSerial.println("AT+TEST=RFCFG,868,SF12,125,8,10,22,ON,OFF,OFF"); //Set LoRa RF configuration
    loraFlush(); //Read and flush message

    millisDelay(100);

    /**********Test***********/

    char message1[50] = {0x01, 0x00, 0x00, 0x00, 0xFF, 0x80, 0x59};

    /************************/
    //Temporary variables for conversion
    char temp1[1];
    char temp2 = 0;

    for (int i = 0; i < messageHead; i++)
    {
      temp2 = messageBuffer[i] >> 4; //Add four MSB to temp2
      sprintf(temp1, "%X", temp2);  //Convert temp2 to hexadecimal representation and add to temp1
      loraTX[(i * 2) + 18] = temp1[0]; //Add temp1 to loraTX message

      temp2 = messageBuffer[i] & 0x0F; //Add four LSB to temp2
      sprintf(temp1, "%X", temp2); //Convert temp2 to hexadecimal representation and add to temp1
      loraTX[(i * 2) + 19] = temp1[0]; //Add temp1 to loraTX message
    }

    memset(messageBuffer, 0, sizeof messageBuffer);

    strcat(loraTX, "\"\r\n"); //Insert ", CR and LF to end of message

    digitalWrite(LED, HIGH); //Turn on LED for message indication

    loraSerial.write(loraTX, strlen(loraTX)); //Write message to LoRa module
    millisDelay(1000); //Wait for message to be sent
    loraRead(); //Read LoRa module response
    memset(readBuffer, 0, sizeof readBuffer); //Empty LoRa buffer

    millisDelay(100);

    digitalWrite(LED, LOW); //Turn of LED

      loraSerial.println("AT+MODE=TEST"); //Enter test mode
      loraRead(); //Read LoRa module response
      memset(readBuffer, 0, sizeof readBuffer); //Empty LoRa buffer

      millisDelay(100);

      loraSerial.println("AT+TEST=RFCFG,868,SF12,125,8,10,22,ON,OFF,OFF"); //Set LoRa RF configuration
      loraRead(); //Read LoRa module response
      memset(readBuffer, 0, sizeof readBuffer); //Empty LoRa buffer

      millisDelay(100);

      loraSerial.println("AT+TEST=RXLRPKT"); //Enter recieve mode
      loraRead(); //Read LoRa module response
      memset(readBuffer, 0, sizeof readBuffer); //Empty LoRa buffer

      millisDelay(100);

      bool flag = false;
      bool flag2 = false;
      while (!flag)
      {
        unsigned long time_now = millis();
        while (!loraSerial.available()) //Wait for message to be recieved
        {
          if (millis() - time_now > 10000) //If no message recieved
          {
            Serial.println("No message received");
            flag2 = true;
            break;
          }
        }

        if (flag2)
        {
		  //Write message to LoRa module
          loraSerial.write(loraTX, strlen(loraTX)); 
          millisDelay(1000);
          loraRead();
          memset(readBuffer, 0, sizeof readBuffer);

		  //Enter setup mode
          loraSerial.println("AT+MODE=TEST");
          loraRead();
          memset(readBuffer, 0, sizeof readBuffer);

          millisDelay(100);

		  //Setup RF configuration
          loraSerial.println("AT+TEST=RFCFG,868,SF12,125,8,10,22,ON,OFF,OFF");
          loraFlush();

          millisDelay(100);

		  //Enter recieve mode
          loraSerial.println("AT+TEST=RXLRPKT");
          loraRead();
          memset(readBuffer, 0, sizeof readBuffer);

          millisDelay(100);

          flag2 = false;
        }
        else
        {
          loraRead();

          char IDtemp[2];

		  //Convert ID to ASCII value
          if((ID >> 4) > 0x09)
          {
            IDtemp[0] = (ID >> 4) + 55;
          }
          else
          {
            IDtemp[0] = (ID >> 4) + 48;
          }

          if((ID & 0x0F) > 0x09)
          {
            IDtemp[1] = (ID & 0x0F) + 55;
          }
          else
          {
            IDtemp[1] = (ID & 0x0F) + 48;
          }

          if (strstr(readBuffer, IDtemp) > 0)
          {
            Serial.println("Message received");
            flag = true;
          }
        }

        memset(readBuffer, 0, sizeof readBuffer);
      }

      millisDelay(100);

	  //Enter low power mode
      loraSerial.println(F("AT+LOWPOWER"));
      loraRead();
      memset(readBuffer, 0, sizeof readBuffer);

    messageHead = 1;
  }
}

void loraRead()
{ // Read response after sending AT command

  millisDelay(200); // Wait for sim module to respons correctly
  int i = 0;

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
  int i = 0;
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

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x01);                    // sends five bytes
  i2c_int.write(0x00);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x1D);                    // sends five bytes
  i2c_int.write(0x00);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x1E);                    // sends five bytes
  i2c_int.write(0x00);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x1F);                    // sends five bytes
  i2c_int.write(0x00);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x2C);                    // sends five bytes
  i2c_int.write(0xFF);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x2D);                    // sends five bytes
  i2c_int.write(0xFF);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x2E);                    // sends five bytes
  i2c_int.write(0x03);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x14);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  i2c_int.requestFrom(GPIO_Adress, 1); //request 1 byte from device #0x68
  int delta_pause = i2c_int.read();

  Serial.print("Delta pause: ");
  Serial.println(delta_pause);

  delta = delta_pause >> 3;

  pause = delta_pause & 0x0F;

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x16);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.requestFrom(GPIO_Adress, 1); //request 1 byte from device #0x68
  short ID_int = i2c_int.read();

  Serial.print("ID1: ");
  Serial.println(delta_pause);

  ID = ID_int << 4;

  millisDelay(100);

  i2c_int.beginTransmission(GPIO_Adress); // transmit to device #0x68
  i2c_int.write(0x15);                    // sends five bytes
  i2c_int.endTransmission();              // stop transmitting

  millisDelay(100);

  i2c_int.requestFrom(GPIO_Adress, 1); //request 1 byte from device #0x68
  ID_int = i2c_int.read();

  Serial.print("ID2: ");
  Serial.println(delta_pause);

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
  else
  {
	interval = 5;
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
  int sensorValue = analogRead(ADC1_LUX);
  
  float voltage = sensorValue * (5.0 / 1023.0);
  
  luxValue = 0.9 * (((10000.0*3.6)/voltage)-10000.0);

  /******dB sensor******/
  dbValue = analogRead(ADC2_DB);

  /******Pir sensor******/
  static int pirTime = 0;
  pirFlag = false;
  if ((millis() - pirTime > pause * 60000) || pirStart)
  {
    pirValue = digitalRead(pirSensor);
    pirTime = millis();
    pirStart = false;
    pirFlag = true;
  }

  /******Temp and humidity******/

  i2c_int.beginTransmission(temp_Adress); // transmit to device #0x44
  i2c_int.write(0x2C);
  i2c_int.write(0x0D);
  i2c_int.endTransmission(); // stop transmitting

  millisDelay(50);

  i2c_int.requestFrom(temp_Adress, 6);
  unsigned char tempHigh = i2c_int.read();
  unsigned char tempLow = i2c_int.read();
  int check1 = i2c_int.read(); // Do not care. Just a place holder
  unsigned char humHigh = i2c_int.read();
  unsigned char humLow = i2c_int.read();
  int check2 = i2c_int.read();

  temp = (tempHigh << 8) | (tempLow);
  humidity = (humHigh << 8) | (humLow);

  temp = (-45.0 + (175.0 * (temp / (powOf(2, 16) - 1.0)))) * 10;
  humidity = (100.0 * (humidity / (powOf(2, 16) - 1.0))) * 10;

  /******Vibration******/
  int vibCounter = 0;
  for (int i = 0; i < 1000000; i++)
  {
    vibValue = digitalRead(Vib_Sensor);
    if (vibValue > 0)
    {
      vibCounter++;
    }
  }

  if (vibCounter > 0)
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

  i2c_int.requestFrom(CO2_Adress, 6);
  unsigned char CO2High = i2c_int.read();
  unsigned char CO2Low = i2c_int.read();
  int check3 = i2c_int.read(); // Do not care. Just a place holder
  int check4 = i2c_int.read();
  int check5 = i2c_int.read(); // Do not care. Just a place holder
  int check6 = i2c_int.read();

  CO2 = (CO2High << 8) | (CO2Low);

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
