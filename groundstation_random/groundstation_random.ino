#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int pinCE = 9;
const int pinCSN = 10;
RF24 radio(pinCE, pinCSN);

// Single radio pipe address for the 2 nodes to communicate.
const uint64_t pipe = 0xE8E8F0F0E1LL;

float data[10];

void setup()
{
	radio.begin();
	Serial.begin(115200);
	radio.openReadingPipe(1, pipe);
	radio.startListening();
}

void loop()
{
  if(true)
	//if (radio.available())
	{
		//radio.read(data, sizeof data);

    data[0] = 1 + data[0];
    data[1] = random(0,10);
		data[2] = random(0,30);
		data[3] = random(0, 1000);
		data[4] = random(0, 1000);
		//data[5] = millis();
		//data[6] = random(0,20);
		Serial.print(data[0] * 3);
		Serial.print(":");
    Serial.print(data[1]);
    Serial.print(":");
    Serial.print(data[2]);
    Serial.print(":");
    Serial.print(data[3]);
    Serial.print(":");
    Serial.print(data[4]);
    //Serial.print(",");
    //Serial.print(data[5]);
    //Serial.print(",");
    //Serial.print(data[6]);
    Serial.println(";");

    delay(100);
	}
	//delay(1000);
}
