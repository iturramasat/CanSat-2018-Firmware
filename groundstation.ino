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
	if (radio.available())
	{
		radio.read(data, sizeof data);
		Serial.print(data[0]);
		Serial.print(",");
    Serial.print(data[1]);
    Serial.print(",");
    Serial.print(data[2]);
    Serial.print(",");
    Serial.print(data[3]);
    Serial.print(",");
    Serial.print(data[4]);
    Serial.print(",");
    Serial.print(data[5]);
    Serial.print(",");
    Serial.print(data[6]);
    Serial.print(",");
    Serial.print(data[7]);
    Serial.print(",");
    Serial.print(data[8]);
    Serial.print(",");
    Serial.print(data[9]);
    Serial.print(";");
	}
	//delay(1000);
}
