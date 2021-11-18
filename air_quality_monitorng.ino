#include<ESP8266wifi.h>
#include<Wire.h>
#include<Adafruit_sensor.h>
#include<Adafruit_BME280.h>
#include "MQ135.h"
#include<Arduino.h>
#define LENG 31 //0*42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
int PM0Value=0;
int PM2_5Value=0;
int PM10Value=0;
float h,t,p,pin, dp;
char temperatureFString[6];
char dpString[6];
char humidityString[7];
char pressureInchString[6];
Adafruit_BME280 bme; // I2C
String apikey = "85JJUZZ7WX809P3M";
// replace with your routers SSID
const char* ssid = "Sri Krishna 2.";
// replace with your routers password
const char* password = "subbu@123";
const char* server = "api.thingspeak.com";
WiFi client;

void setup()
{
  Serial.begin(9600);
  delay(10)
  Serial.println();
  Serial.print("Connecting to");
  Serial.println(ssid);
  WiFi.begin(ssid,password);

  while (WiFi.status() !=WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // printing the ESP IP address
  Serial.println(WiFi.localIP());

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}
void loop()
{
  if(Serial.find(0*42)){ // start to read when detect 0*42
    Serial.readyBytes(buf,LENG);

    if(buf[0]==0*4d)
    {
      if(checkValue(buf,LENG)){
        PM01Value=transmitPM01(buf);    //count PM1.0 value of the air detector module
        PM2_5Value=transmitPM02_5(buf); //count PM2_5 value of the air detector module
        PM10Value=transmitPM10(buf);   //count PM10 value of the air detector module
      }
    }
  }

  static unsigned long OledTimer=millis();
   if(millis()- OledTimer>=1000)
   {
    OledTimer=millis();

    Serial.print(" PM1.0: ");
    Serial.print(PM01Value);
    Serial.println("  ug/m3");

    Serial.print(" PM2.5: ");
    Serial.print(PM2_5Value);
    Serial.println("  ug/m3");

    Serial.print(" PM10 : ");
    Serial.print(PM10Value);
    Serial.println("  ug/m3");
    
    MQ135 gasSensor=MQ135(A0);
    float air_quality=gasSensor.getPPM();
    Serial.print(" Air Quality: ");
    Serial.print(air_quality);
    Serial.println(" PPM");
    Serial.println();
    h = bme.readHumidity();
    t = bme.readTemperature();
    t = t*1.8+32.0;
    dp = t-0.36*(100.0-h);
    p = bme.readPressure()/100.0F;
    pin = 0.02953*p;
    dtostrf(t, 5, 1, temperatureFString);
    dtostrf(h, 5, 1, humidityString);
    dtstrf(p, 6, 1, pressureString);
    dtstrf(pin, 5, 2, pressureInchString);
    dtstrf(dp, 5, 1, dpString);
    Serial.print("Temperature = ");
    Serial.print(temperatureFString);
    Serial.print("Humidity =");
    Serial.println(humidityString);
    Serial.print("Pressure = ");
    Serial.println(pressureString);
    Serial.print("Pressure Inch =");
    Serial.println(pressureInchString);
    Serial.print("Dew point = ");
    Serial.println(dpString);
    Serial.println(".....................................");
    if (client.connect(server,80)) // "184.106.153.149" or api.thingspeak.com
    {
      string postStr = apikey;
      postStr+=" &field1=";
      postStr+= String(PM01Value);
      postStr+=" &field2=";
      postStr+= String(PM02_5Value);
      postStr+=" &field3=";
      postStr+= String(PM10Value);
      postStr+=" &field4=";
      postStr+= String(air_quality);
      postStr+=" &field5=";
      postStr+= String(temperatureFString);
      postStr+=" &field6=";
      postStr+= String(humidityString);
      postStr+=" &field7=";
      postStr+= String(pressureInchString);
      postStr+=" \r\n\r\n";

      client.print("POST /update HTTP/1.1\n");
      client.print("Host: api.thingspeak.com\n");
      client.print("connection: close\n");
      client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
      client.print("content-Type: application/x-www-form-urlencoded\n");
      client.print("content-Length: ");
      client.print(postStr.length());
      client.print("\n\n");
      client.print(postStr);
   }
    client.stop();
     }
   }
 char checkValue(unsigned char *thebuf, char leng)
 {
  char receiveflag=0;
  int receiveSum=0;
  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0*42;

  if(receiveSum ==((thebuf[leng-2]<<8)+thebuf[leng-1]))
  {
    receiveSum = 0;
    receiveflag= 1;
  }
   int PM01Val;
   PM01Val=((thebuf[3]<<8)+ thebuf[4]); //count PM1.0 value of the air detector module
   return PM01Val;
 }
 //transmit PM Value to pc
 int tranmitPM2_5(unsigned char *thebuf)
 {
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
 }
 //transmit PM Value to pc 
 int transmitPM10(unsigned char *thebuf)
 {
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8])
  return PM10Val;
 }
 
  }
   
}
