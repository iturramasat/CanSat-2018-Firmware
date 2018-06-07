//BMP
#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 bmp180;

double Po; //presion del punto inicial para h=0;
char status;
double Tbmp, Pbmp, Hbmp;
float Hforbmp;

//MPX+LM35
float Pmpx, Hfor, Tlm35;

double T1 = 298; //Lur mailako presioa, tenperatura eta altuera guk zehaztuta
double P1 = 96700; //double???????
double h1 = 0;

//GPS
#include <TinyGPS.h>
//#include <SoftwareSerial.h>

TinyGPS gps;

#define gpsPort Serial1

char buf[32];

float gps_alt;
float flat, flon;

//RADAR
#define Ntar 5

uint8_t results[Ntar * 12], buff_temp[4];
float range[Ntar], velocity[Ntar], SNR[Ntar];
uint8_t i, bytesToRead;
unsigned long t1, t2;

float maiztasuna;

//APC
String mystringBidali, mystringGorde;
int counter = 0;

//Servo
#include <Servo.h>
Servo servo1;
Servo servo2;
int parakaidas_ireki = 0;
int parakaidas_itxi = 45;

void setup()
{
  Serial.begin(115200);
  Serial.println("Seriala irekita");
  gpsPort.begin(9600);
  Serial2.begin(115200);//SD
  Serial3.begin(115200);//RADAR

  while (Serial3.available()) {
    Serial3.read();
  }

  //BMP
  if (bmp180.begin())
  {
    Serial.println("BMP180 iniciado");
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
            Serial.println("Punto de referencia establecido: h=0");
          }
        }
      }
    }

  }
  else
  {
    Serial.println("Error al iniciar el BMP180");
    //while (1);
  }

  //Servo
  servo1.attach(22);
  servo2.attach(23);

}

void loop()
{
  //unsigned long t0 = millis();
  //BMP
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
          Serial.print("Hbmp= ");
          Serial.print(Hbmp);
          Serial.println(" m");
        }
      }
    }
  }
  Serial.print("Pbmp= ");
  Serial.print(Pbmp * 100);
  Serial.println(" Pascal");
  Serial.print("Tbmp= ");
  Serial.print(Tbmp);
  Serial.println(" C");

  Hforbmp = (((T1 /  -0.0065) * ((pow(((Pbmp * 100) / P1), (0.0065 * 287.06 / 9.81))) - 1)) + h1);

  Serial.print("Hforbmp= ");
  Serial.print(Hforbmp);
  Serial.println(" m");

  //MPX
  Pmpx = ((((analogRead(A1) * (3.3 / 3.18)) / 1024) + 0.095) / 0.000009);

  Serial.print("Presioa MPX = ");
  Serial.print(Pmpx);
  Serial.println(" Pascal");

  Hfor = (((T1 /  -0.0065) * ((pow(((Pmpx) / P1), (0.0065 * 287.06 / 9.81))) - 1)) + h1);

  Serial.print("Altuera formula: ");
  Serial.print(Hfor);
  Serial.println(" m");

  //LM35
  int analogValue = analogRead(A0);
  float millivolts = (analogValue / 1023.0) * 3300;
  Tlm35 = millivolts / 10;
  Serial.print(Tlm35);
  Serial.println("º LM35");


  //GPS
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  //for (unsigned long start = millis(); millis() - start < 1000;) {
  while (gpsPort.available()) {
    char c = gpsPort.read();
    // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    if (gps.encode(c)) // Did a new valid sentence come in?
      newData = true;
  }
  //}

  unsigned long age;
  if (newData) {
    gps_alt = gps.f_altitude();
    Serial.print("GPS ALTUERA: ");
    Serial.print(gps_alt);
    Serial.println(" m");
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    //GPS mode
    Serial.print(" Constellations=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0 : gps.constellations());
  }

  //satellites in view
  uint32_t* satz = gps.trackedSatellites();
  uint8_t sat_count = 0;
  for (int i = 0; i < 24; i++) {
    if (satz[i] != 0) {  //exclude zero SNR sats
      sat_count++;
      byte strength = (satz[i] & 0xFF) >> 1;
      byte prn = satz[i] >> 8;
      sprintf(buf, "PRN %d: ", prn);
      Serial.print(buf);
      Serial.print(strength);
      Serial.println("dB");
    }
  }

  //date time
  int year;
  uint8_t month, day, hour, minutes, second, hundredths;
  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &age);
  sprintf(buf, "GPS time: %d/%02d/%02d %02d:%02d:%02d", year, month, day, hour, minutes, second);
  Serial.println(buf);

  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

  //RADAR
  delay(10);

  t1 = millis();

  Serial3.write(255);

  bytesToRead = Serial3.available();
  while (bytesToRead < 12 * Ntar) {
    bytesToRead = Serial3.available();
  }

  t2 = millis() - t1;

  i = 0;
  while (Serial3.available() && i < 12 * Ntar) {
    results[i] = Serial3.read();
    i++;
  }

  for (i = 0; i < Ntar; i++) {
    buff_temp[0] = results[i * 4];
    buff_temp[1] = results[i * 4 + 1];
    buff_temp[2] = results[i * 4 + 2];
    buff_temp[3] = results[i * 4 + 3];
    range[i] = *((float*)(buff_temp));

    buff_temp[0] = results[Ntar * 4 + i * 4];
    buff_temp[1] = results[Ntar * 4 + i * 4 + 1];
    buff_temp[2] = results[Ntar * 4 + i * 4 + 2];
    buff_temp[3] = results[Ntar * 4 + i * 4 + 3];
    velocity[i] = *((float*)(buff_temp));

    buff_temp[0] = results[Ntar * 8 + i * 4];
    buff_temp[1] = results[Ntar * 8 + i * 4 + 1];
    buff_temp[2] = results[Ntar * 8 + i * 4 + 2];
    buff_temp[3] = results[Ntar * 8 + i * 4 + 3];
    SNR[i] = *((float*)(buff_temp));
  }
  for (i = 0; i < Ntar; i++) {
    Serial.print("### Target ");
    Serial.print(i + 1);
    Serial.println(" ###");

    Serial.print("Range: ");
    Serial.print(range[i]);
    Serial.println(" m");

    Serial.print("Velocity: ");
    Serial.print(velocity[i]);
    Serial.println(" m/s");

    Serial.print("SNR: ");
    Serial.print(SNR[i]);
    Serial.println(" dB");
    Serial.println("");
  }
  Serial.println(t2);
  Serial.println("");

  //APC
  counter = counter + 1;
  maiztasuna = counter * 1000 / millis();
  Serial.print("Maiztasuna: ");
  Serial.print(maiztasuna);
  Serial.println(" pakete/s");

  mystringBidali = String(counter) + ":" + String(Pbmp) + ":" + String(Tbmp) + ":" + String(Hbmp) + ":" +  String(Hforbmp) + ":" +  String(range[0]) + ":" + String(Tlm35) + ":" + String(Pmpx / 100) + ":" + String(Hfor) + ":" + String(flat * 100000) + ":" + String(flon * 100000) + ":" + String(gps_alt) + ";";

  mystringGorde = String(counter) + ":" + String(millis()) + ":" + String(Pbmp) + ":" + String(Tbmp) + ":" + String(Hbmp) + ":" +  String(Hforbmp) + ":" +  String(range[0]) + ":" + String(Tlm35) + ":" + String(Pmpx / 100) + ":" + String(Hfor) + ":" + String(flat * 1000) + ":" + String(flon * 1000) + ":" + String(gps_alt) + ";";

  Serial1.println(mystringBidali);
  Serial2.println(mystringGorde);
  Serial.println(mystringBidali);

  //Servo
  
  if (Serial.available()) {
    char mezua = Serial.read();
    if (mezua == 'A') {
      servo1.write(parakaidas_ireki);
      servo2.write(parakaidas_ireki);
    } else if (mezua == 'B') {
      servo1.write(parakaidas_itxi);
      servo2.write(parakaidas_itxi);
    }
  }
}
