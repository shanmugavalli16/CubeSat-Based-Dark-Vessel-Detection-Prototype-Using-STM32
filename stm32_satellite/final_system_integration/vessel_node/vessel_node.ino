#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9,10);

const byte address[6] = "00001";

void setup()
{
  Serial.begin(115200);

  if (!radio.begin())
  {
    Serial.println("NRF not found");
    while (1);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPayloadSize(32);

  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("Arduino TX Ready");
}
char text[32];
int count = 0;

void loop()
{
    sprintf(text, "HB:%d", count++);

    radio.write(text, 32);

    Serial.println(text);

    delay(1000);
}
