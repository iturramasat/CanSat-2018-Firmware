#include <SFE_BMP180.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

SoftwareSerial gpsSerial(2, 3); // RX, TX
TinyGPS gps;

SFE_BMP180 bmp180;

double Po; //presion del punto inicial para h=0
char status;
double Tbmp, Pbmp, Hbmp;

float T1 = 293;//Lur mailako presioa, tenperatura eta altuera
float P1 = 96724;
float h1 = 0;

#include <SD.h>

File myFile;

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
 
void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  if (bmp180.begin())
  {
    Serial.println("BMP180 piztua");
    status = bmp180.startTemperature(); //Inicio de lectura de temperatura
    if (status != 0)
    {
      delay(status); //Pausa para que finalice la lectura
      status = bmp180.getTemperature(Tbmp);//Obtener la temperatura
      if (status != 0)
      {
        status = bmp180.startPressure(3); //Inicio lectura de presión
        if (status != 0)
        {
          delay(status); //Pausa para que finalice la lectura
          status = bmp180.getPressure(Pbmp, Tbmp); //Obtener la presión
          if (status != 0)
          {
            Po = Pbmp; //Asignamos el valor de presión como punto de referencia
            Serial.println("Erreferentzia puntua: h=0");
          }
        }
      }
    }
  }
  else
  {
    Serial.println("BMP180 pizteko arazoak");
    while (1);
  }

  Serial.print("SD pizten ...");
  if (!SD.begin(4)) 
  {
    Serial.println("SD erabilarazteko arazoak");
    return;
  }
  Serial.println("SD erabilarazia");//ARRAROA????????????

    
   radio.begin();
   radio.openWritingPipe(pipe);

}

long lat, lon;
unsigned long fix_age, gps_time, date, gps_speed, course;//COURSEREN KOLOREA ????????????
unsigned long chars;
unsigned short sentences, failed_checksum;
float gps_alt;


void loop()
{ 
  
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

  status = bmp180.startTemperature(); //Inicio de lectura de temperatura
  if (status != 0)
  {
    delay(status); //Pausa para que finalice la lectura
    status = bmp180.getTemperature(Tbmp); //Obtener la temperatura
    if (status != 0)
    {
      status = bmp180.startPressure(3); //Inicio lectura de presión
      if (status != 0)
      {
        delay(status); //Pausa para que finalice la lectura
        status = bmp180.getPressure(Pbmp, Tbmp); //Obtener la presión
        if (status != 0)
        {
          Hbmp = bmp180.altitude(Pbmp, Po); //Calcular altura con respecto al punto de referencia

          Serial.println();

          Serial.print("Presioa BMP180=");
          Serial.print(Pbmp * 100);
          Serial.println("Pascal");

          Serial.print("Tenperatura BMP180=");
          Serial.print(Tbmp);
          Serial.println("ºC");

          Serial.print("Altuera BMP180=");
          Serial.print(Hbmp);
          Serial.println(" m");
        }
      }
    }
  }
    int value = analogRead(A3);
    float millivolts = (value / 1023.0) * 5000;
    float Tlm35 = millivolts / 10; 
    Serial.print("Tenperatura lm35= ");
    Serial.print(Tlm35);
    Serial.println(" ºC");
    
    float Pmpx = readPressure(A1);
    float millibars = Pmpx/100;
 
    Serial.print("Presioa MPX= ");
    Serial.print(Pmpx);
    Serial.println(" Pascal");
    Serial.print("Presioa MPX= ");
    Serial.print(millibars);
    Serial.println(" milibar");
    
    Hfor = (((T1/  -0.0065)*((pow(((Pmpx)/P1),(0.0065*287.06/9.81)))-1))+h1);
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
float readPressure(int pin)
{
    int pressureValue = analogRead(pin);
    float Pmpx=((pressureValue/1024.0)+0.095)/0.000009;
    return Pmpx;}
