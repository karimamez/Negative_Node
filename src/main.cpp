#include <Arduino.h>

#include <SPI.h>
#include <LoRa.h>
#include <base64.h>
//Libraries for MAX31865
#include <Adafruit_MAX31865.h>
//Libraries for BME280
#include <Adafruit_BMP280.h>



//define the pins used by the LoRa transceiver module
#define SS    GPIO_NUM_18
#define RST   GPIO_NUM_14
#define DIO0  GPIO_NUM_26

#define BAND 868E6


//BME280 definition
#define BMP_SCK 10
#define BMP_MISO 36
#define BMP_MOSI 12
#define BMP_CS 9   
Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK); 



// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(15, 12, 36, 10); 
#define RREF      430.0
#define RNOMINAL  100.0

#define uS_TO_S_FACTOR 1000000  
#define TIME_TO_SLEEP  1198   ////1200 s = 20min  

float XS = 0.00225;     
uint16_t MUL = 1000;
uint16_t MMUL = 100;


byte Net_ID=0x06;
byte MSG_ID=0;
String MSG;
String ID="NN";
char space=' ';
base64 base64Object;
String encodedMessage;

float temperature = 0;
float humidity = 0;
float pressure = 0;

float temp = 0;
float battery = 0;

uint16_t voltage;

RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR int bootCount = 0;

uint8_t DevEUI[8] = {0x9a, 0x38, 0x33, 0x68, 0x7c, 0x41, 0xbc, 0x69};

void getReadings();
void sendReadings();
int Sum ( String Message );
int Normalize ( int P );
int Request_ByteID ( int t );


void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  //////////////////////////////////////////////////////////////////////////

    
  delay(50);
  ///////////////////////////////////////////////////////////////////////////
  
}
void loop() {
  
  getReadings();
  sendReadings();
  
  ++bootCount;     
  Serial.println("Boot: " + String(bootCount));

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("going to sleep " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.println("Going to sleep now");

  LoRa.end();
  LoRa.sleep();
  delay(10);

  digitalWrite(Vext, HIGH);
  delay(10);
  Serial.flush(); 
  esp_deep_sleep_start();
  
}


void getReadings()
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  delay(50);

  voltage  =  ( analogRead(37)*XS*MUL )+ 950;
  battery  = float(voltage)/1000;
  Serial.print("Battery = ");
  Serial.print(battery);
  Serial.println("V");

  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  temp = thermo.temperature(RNOMINAL, RREF);
  Serial.print("Temperature PT100 = "); 
  Serial.print(temp);
  Serial.println("ºC");

  bme.begin();  
  temperature = bme.readTemperature();
  Serial.print("Temperature BME = "); 
  Serial.print(temperature);
  Serial.println("ºC");

  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void sendReadings()
{
  
  MSG = ID + space + String(temp) + space + String(temperature) + space + String(battery) + space + "80" + '\n'; 

  LoRa.setSPI(SPI);
  LoRa.setPins(SS, RST, DIO0);
  LoRa.begin(BAND);
    
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.write(Net_ID);

  encodedMessage = base64Object.encode(String(DevEUI, 8) + MSG);
  encodedMessage += "\n";
  MSG_ID = Request_ByteID(Normalize(Sum(encodedMessage)));

  LoRa.write(MSG_ID);
  encodedMessage = encodedMessage + space;
  LoRa.print(encodedMessage);
  LoRa.endPacket();

  Serial.print("Net_ID=");
  Serial.println(Net_ID);
  Serial.print("MSG_ID=");
  Serial.println(MSG_ID);
  Serial.print("MSG=");
  Serial.println(encodedMessage); 

  counter++;
}


int Sum ( String Message ) {
  int SUM=0;
  for (int w=0; w<Message.length(); w++){ 
    SUM += byte(Message[w]);
  }
  return SUM;
}


int Normalize ( int P ) {
  if ( P > 288 ) {
    do {
      P = P - 256;
      } while( P > 288 );
    }
  return P;
}


int Request_ByteID ( int t ) {
  if(t <= 32)
  {
    return (32 - t);
  }
  else
  {
    return (288 - t);
  }
}