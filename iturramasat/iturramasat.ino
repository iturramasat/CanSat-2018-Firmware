#include <SFE_BMP180.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

SoftwareSerial gpsSerial(2, 3); // RX, TX
TinyGPS gps;

SFE_BMP180 bmp180;

double Po; //presion del punto inicial para h=0;
char status;
double T, P, A;

float T1 = 293;
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
const uint64_t pipe = 0xE8E8F0F0E1LL;

float data[9];
int counter = 0;

void setup(){
  Serial.begin(9600);
  gpsSerial.begin(9600)
  if (bmp180.begin())
  {
    Serial.println("BMP180 piztua");
    status = bmp180.startTemperature(); //Inicio de lectura de temperatura
    if (status != 0)
    {
      delay(status); //Pausa para que finalice la lectura
      status = bmp180.getTemperature(T);//Obtener la temperatura
      if (status != 0)
      {
        status = bmp180.startPressure(3); //Inicio lectura de presión
        if (status != 0)
        {
          delay(status); //Pausa para que finalice la lectura
          status = bmp180.getPressure(P, T); //Obtener la presión
          if (status != 0)
          {
            Po = P; //Asignamos el valor de presión como punto de referencia
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
  if (!SD.begin(4)) {
    Serial.println("SD erabilarazteko arazoak");
    return;
  }
  Serial.println("SD erabilarazia");
}

long lat, lon;
unsigned long fix_age, gps_time, date, gps_speed, course;
unsigned long chars;
unsigned short sentences, failed_checksum;

   radio.begin();
   radio.openWritingPipe(pipe);
}

void loop(){
  bool ready = false;
  if (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
    if (gps.encode(c)) {
      // retrieves +/- lat/long in 100000ths of a degree
      gps.get_position(&lat, &lon, &fix_age);

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
    status = bmp180.getTemperature(T); //Obtener la temperatura
    if (status != 0)
    {
      status = bmp180.startPressure(3); //Inicio lectura de presión
      if (status != 0)
      {
        delay(status); //Pausa para que finalice la lectura
        status = bmp180.getPressure(P, T); //Obtener la presión
        if (status != 0)
        {
          A = bmp180.altitude(P, Po); //Calcular altura con respecto al punto de referencia

          Serial.println();

          Serial.print("presioa=");
          Serial.print(P * 100);
          Serial.println("paskal");

          Serial.print("tenperatura=");
          Serial.print(T);
          Serial.println("ºC");

          Serial.print("altuera=");
          Serial.print(A);
          Serial.println(" m");

          double h;
          h = (((T1 /  -0.0065) * ((pow(((P * 100) / P1), (0.0065 * 287.06 / 9.81))) - 1.0)) + h1);
          Serial.print("Altuera: ");
          Serial.print(h);
          Serial.println(" m");
          Serial.println();
          delay(1000);

          Serial.println();
        }
      }
    }
  }
  delay(1000);

  myFile = SD.open("datalog.txt", FILE_WRITE);//abrimos  el archivo

  if (myFile) {
    Serial.print("SD idazten: ");
/*
 *
long lat, lon;
unsigned long fix_age, gps_time, date, gps_speed, course;
unsigned long chars;
unsigned short sentences, failed_checksum;

 */
    //myFile.print("Tiempo(ms)=");
    myFile.print(millis());
    myFile.print(",");
    myFile.print(P);
    myFile.print(",");
    myFile.print(T);
    myFile.print(",");
    myFile.print(A);
    myFile.print(",");
    myFile.print(h);
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

    myFile.close(); //cerramos el archivo

    Serial.print("Denbora(ms)=");
    Serial.print(millis());
    Serial.print(", sensor1=");
    Serial.print(P);
    Serial.print(", sensor2=");
    Serial.print(T);


  } else {
    Serial.println("Fitxategia irekitzeko arazoak");
  }
  delay(100);

   data[0] = counter;
   data[1] = P; // Milibar
   data[2] = T; // Celsius
   //data[3] = A; // Altuera
   //data[4] = h; //
   data[3] = lat; // UTM
   data[4] = lon; // UTM
   data[5] = date; // .. ToDo: Test
   data[6] = gps_time; // .. ToDo: Test
   data[7] = gps_speed; // ToDo: Test

   radio.write(data, sizeof data);
   counter = counter + 1;
   delay(1000);
}
