#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);

const byte address[6] = "00001";

char text[32];

void setup()
{
  Serial.begin(115200);

  if (!radio.begin())
  {
    Serial.println("NRF NOT FOUND");
    while (1);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);

  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("Receiver Ready");
}

void loop()
{
  if (radio.available())
  {
    radio.read(&text, sizeof(text));

    Serial.print("Received: ");
    Serial.println(text);
  }
}
