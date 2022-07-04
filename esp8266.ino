#define BLYNK_TEMPLATE_ID "TMPLMzk2pkcO"
#define BLYNK_DEVICE_NAME "BMS"
#define BLYNK_AUTH_TOKEN "FJSeFiexgG1Iqna3gm71vI_y9zQyj7r_"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI


#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "ACS712.h"
#include "BlynkEdgent.h"

SoftwareSerial mySerial(14, 12); //(Txd,Rxd) GSM module
SoftwareSerial ss(13, 15); //(Txd,Rxd) GPS module

TinyGPSPlus gps;
ACS712  ACS(A0, 3.3, 1023, 66);
#define s0 16
#define s1 5
#define s2 4
#define s3 0
#define capacity 1950
#define rcapacity 1950





const int voltageSensorPin = A0;           // sensor pin
int mA = 0;
float vIn;                                 // measured voltage (3.3V = max. 16.5V, 5V = max 25V)
float vOut;
float voltageSensorVal;                    // value on pin A3 (0 - 1023)
const float factor = 5.128;                // reduction factor of the Voltage Sensor shield
const float vCC = 3.3;                     // Arduino input voltage (measurable by voltmeter)
float calc = 0.0;
float volt = 0.0;
float bat_percentage = 100.0;
float temp_percentage = 0.0;
float TotalAh = 0.0;
const float calibration = -0.12;           // Calibration factor
String apiKey = "D02A7I7XIW0RSYEX";


float SOC;
float SOH;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const int period = 1000;  //the value is a number of milliseconds
float totalCoulumbs = 0.0;


const char *ssid = "hacker"; // replace with your wifi ssid and wpa2 key
const char *pass = "praveen01";
const char* server1 = "api.thingspeak.com";
int f1 = 0, f2 = 0, f3 = 0, f4 = 0, flag = 0;
//String message = "Hi, Renolin";
WiFiClient client;

void setup()
{
  Serial.begin(9600);
  delay(10);
  mySerial.begin(9600);
  ss.begin(9600);
  BlynkEdgent.begin();

  mySerial.println("AT");
  updateSerial();
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startMillis = millis();
  ACS.autoMidPoint();
}



String gpsInfo()
{
  int gflag = 0;
  String gdata;
  Serial.print(F("Location: "));
  //while (ss.available() > 0 && gps.encode(ss.read()) && gflag == 0)
  //{
    if (gps.location.isValid())
    {
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
      gdata = String(gps.location.lat()) + "," + String(gps.location.lng());
      gflag = 1;
      return gdata;
    }
    else
    {
      Serial.println("INVALID");
      gflag = 0;
      return "INVALID";
    }
//  }
}



void sendsms(String msg)
{

  // Configuring module in TEXT mode
  mySerial.println("AT+CMGF=1");
  updateSerial();

  // to send message use these 3 statements, upto write(26)
  // change ZZ with country code and xxxxxxxxxxx with phone number to sms
  mySerial.println("AT+CMGS=\"+916382102127\""); // 1)
  updateSerial();
  String s = msg + "Loction:https://maps.google.com/?q=" + gpsInfo();
  mySerial.print(s);
  updateSerial();
  mySerial.write(26);

}
float getAnalog_Voltage(int s_pin)
{
  digitalWrite(s3, HIGH && (s_pin & B00001000));
  digitalWrite(s2, HIGH && (s_pin & B00000100));
  digitalWrite(s1, HIGH && (s_pin & B00000010));
  digitalWrite(s0, HIGH && (s_pin & B00000001));
  return (float)analogRead(A0);
}

float getAnalog_Current(int s_pin)
{
  digitalWrite(s3, HIGH && (s_pin & B00001000));
  digitalWrite(s2, HIGH && (s_pin & B00000100));
  digitalWrite(s1, HIGH && (s_pin & B00000010));
  digitalWrite(s0, HIGH && (s_pin & B00000001));
  return (float)ACS.mA_DC();
}

void loop() {

  BlynkEdgent.run();
  voltageSensorVal = getAnalog_Voltage(15);  // read the current sensor value (0 - 1023)
  vOut = (voltageSensorVal / 1024) * vCC;             // convert the value to the real voltage on the analog pin
  vIn =  (vOut * factor) + calibration;
  bat_percentage = mapfloat(vIn, 2.5, 8.2, 0, 100);

  if (bat_percentage >= 100.00) {
    bat_percentage = 100.00;
  }
  else if (bat_percentage <= 0.0) {
    bat_percentage = 0.0;
  }
  mA = getAnalog_Current(14);     // read the current sensor value (0 - 1023)
  Serial.print("Voltage = ");
  Serial.print(vIn);
  Serial.println("V");
  Serial.print(mA);
  Serial.println("A");

  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    totalCoulumbs = totalCoulumbs + mA;
    TotalAh = totalCoulumbs / 3600.0;
    Serial.print("Capacity = ");
    Serial.println(capacity - TotalAh);
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current state.
  }
  SOC = ((capacity - TotalAh) / rcapacity) * 100;
  Serial.print(bat_percentage);
  Serial.println("%");
  Serial.print(SOC);
  Serial.println("%");
  Serial.println("\n");
  delay(1000);
  if (client.connect(server1, 80)) // "184.106.153.149" or api.thingspeak.com
  {
    String postStr = apiKey + "&field1=" + String(vIn) + "&field2=" + String(bat_percentage) + "&field3=" + String(capacity - TotalAh) + "&field4" + String(mA) + "\r\n\r\n";

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
    Serial.println("Data Send to Thingspeak");
  }
  Blynk.virtualWrite(V0, bat_percentage);
  Blynk.virtualWrite(V1, vIn);
  Blynk.virtualWrite(V2, mA);
  if (bat_percentage > 70 && bat_percentage <= 98 && flag == 0 )
  {
    flag = 1;
    Serial.println("flag 1");
    sendsms("Your battery level is above 70% !!!");
    Serial.println("SMS send succesfully !");

  }
  if (bat_percentage >= 40.00 && bat_percentage < 50.00 && f1 == 0) {
    f1 = 1;
    //    mySerial.write(26); // 3)
    sendsms("Your battery level is below 50% !!!");
    Serial.println("SMS send succesfully !");
  }
  else if (bat_percentage >= 30.00 && bat_percentage < 40.00 && f2 == 0) {
    f2 = 1;
    sendsms("Your battery level is below 30% !!!");
    Serial.println("SMS send succesfully !");
  }
  else if (bat_percentage >= 20.00 && bat_percentage < 30.00 && f4 == 0) {
    f3 = 1;
    //    mySerial.write(26); // 3)
    sendsms("Your battery is almost dead! plug in the charger.");
    Serial.println("SMS send succesfully !");
  }
  else if (bat_percentage >= 70)
  {
    f1 = 0;
    f2 = 0;
    f3 = 0;
    f4 = 0;
  }
  delay(500);
  client.stop();
  Serial.println("Waiting...");
  // thingspeak needs minimum 15 sec delay between updates.
  delay(1500);

}

float mapfloat(float v, float in_min, float in_max, float out_min, float out_max)
{
  return ((v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void updateSerial() {
  delay(500);
  while (Serial.available()) {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }

  while (mySerial.available()) {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}
