/*

   GPS

*/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
SoftwareSerial gpsSerial(2, 3); // RX, TX
TinyGPS gps;

long lat, lon;
unsigned long fix_age, gps_time, date, gps_speed, course;//COURSEREN KOLOREA ????????????
unsigned long chars;
unsigned short sentences, failed_checksum;
float gps_alt;


/*

   BMP180

*/

#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

double Po; //presion del punto inicial para h=0
char status;
double Tbmp, Pbmp, Hbmp;


/*

   SD TXARTELA

*/

#include <SD.h>
File myFile;


/*

    NRF24l01

*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int pinCE = 9;
const int pinCSN = 10;
RF24 radio(pinCE, pinCSN);

// Single radio pipe address for the 2 nodes to communicate.
const uint64_t pipe = 0xA8E8F0F0E1LL;
float data[9];
int counter = 0;

/*

   MPX

*/

float readPressure(int pin) {
  int pressureValue = analogRead(pin);
  float Pmpx = ((pressureValue / 1024.0) + 0.095) / 0.000009;
  return Pmpx;
}

/*

   Besteak

*/

float T1 = 293;//Lur mailako presioa, tenperatura eta altuera
float P1 = 96724;
float h1 = 0;


void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  } else {
    Serial.println("BMP Sentsorea piztuta");
  }

  Serial.print("SD pizten ...");
  if (!SD.begin(4)) {
    Serial.println("SD erabilarazteko arazoak");
    //return;
  } else {
    Serial.println("SD erabilarazia");//ARRAROA????????????
  }


  radio.begin();
  radio.openWritingPipe(pipe);

}



void loop()
{
  readGroundStation();
  double Hfor;

  if (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);
    if (gps.encode(c)) {
      // retrieves +/- lat/long in 100000ths of a degree
      gps.get_position(&lat, &lon, &fix_age);
      // +/- altitude in meters
      gps_alt = gps.f_altitude();
      // time in hhmmsscc, date in ddmmyy
      gps.get_datetime(&date, &gps_time, &fix_age);

      // returns speed in 100ths of a knot
      gps_speed = gps.speed();

      // course in 100ths of a degree
      course = gps.course();
    }
  }

  Tbmp = bmp.readTemperature();
  Pbmp = bmp.readPressure();
  Hbmp = bmp.readAltitude();
  
  int value = analogRead(A3);
  float millivolts = (value / 1023.0) * 5000;
  float Tlm35 = millivolts / 10;
  Serial.print("Tenperatura lm35= ");
  Serial.print(Tlm35);
  Serial.println(" ºC");

  float Pmpx = readPressure(A1);
  float millibars = Pmpx / 100;

  Serial.print("Presioa MPX= ");
  Serial.print(Pmpx);
  Serial.println(" Pascal");
  Serial.print("Presioa MPX= ");
  Serial.print(millibars);
  Serial.println(" milibar");

  Hfor = (((T1 /  -0.0065) * ((pow(((Pmpx) / P1), (0.0065 * 287.06 / 9.81))) - 1)) + h1);
  Serial.print("Altuera formula: ");
  Serial.print(Hfor);
  Serial.println(" m");
  Serial.println();

  myFile = SD.open("datalog.txt", FILE_WRITE);//abrimos  el archivo

  if (myFile) {
    Serial.print("SD idazten: ");

    //myFile.print("Tiempo(ms)=");
    myFile.print(millis());
    myFile.print(",");
    myFile.print(Pbmp);
    myFile.print(",");
    myFile.print(Tbmp);
    myFile.print(",");
    myFile.print(Hbmp);
    myFile.print(",");
    myFile.print(Hfor);
    myFile.print(",");
    myFile.print(lat);
    myFile.print(",");
    myFile.print(lon);
    myFile.print(",");
    myFile.print(date);
    myFile.print(",");
    myFile.print(gps_time);
    myFile.print(",");
    myFile.print(gps_speed);
    myFile.print(",");
    myFile.print(gps_alt);

    myFile.close(); //cerramos el archivo

  } else {
    Serial.println("Fitxategia irekitzeko arazoak");
  }

  data[0] = counter;
  data[1] = Pbmp;
  data[2] = Tbmp;
  data[3] = Pmpx;
  data[4] = Tlm35;
  data[5] = lat;
  data[6] = lon;
  data[7] = gps_alt;
  data[8] = gps_time;
  data[9] = gps_speed;

  radio.write(data, sizeof data);
  counter = counter + 1;
}

/*
 * Komunikazio bidirekzionala programatzen hasi..
 */

int send_delay;
boolean enable_accel, cam1_recording, cam2_recording;

void readGroundStation(){
  if (radio.available())
   {    
      radio.read(data, sizeof data);
      /*  1. send_delay = Delay between sends (int)
       *  2. enable_accel = Accelerometer on/off (bool)
       *  3. cam1_recording = Camara 1 status (Recording on/off)
       *  4. cam2_recording = Camara 2 status (Recording on/off)
       */
      send_delay = data[0];
      enable_accel = data[1];
      cam1_recording = data[2];
      cam2_recording = data[3];
   }
}
