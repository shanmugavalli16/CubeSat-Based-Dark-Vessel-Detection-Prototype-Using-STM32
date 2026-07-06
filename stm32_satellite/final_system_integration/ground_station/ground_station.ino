/*
   GROUND STATION (Arduino UNO)
   Receives telemetry from STM32

   Address : "00002"
*/

#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);      // CE, CSN

const byte address[6] = "00002";

char rxData[32];

int vesselStatus;
float pitch, roll, yaw;

void setup()
{
  Serial.begin(9600);

  if (!radio.begin())
  {
    Serial.println("NRF24 NOT FOUND!");
    while (1);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(false);
  radio.setPayloadSize(32);

  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("=================================");
  Serial.println("GROUND STATION READY");
  Serial.println("Waiting for telemetry...");
  Serial.println("=================================");
}

void loop()
{
  if (radio.available())
  {
    memset(rxData, 0, sizeof(rxData));
    radio.read(rxData, sizeof(rxData));

    Serial.println("--------------------------------");
    Serial.print("Raw Packet: ");
    Serial.println(rxData);

    char *p;

    p = strstr(rxData, "V:");
    if (p) vesselStatus = atoi(p + 2);

    p = strstr(rxData, "P:");
    if (p) pitch = atof(p + 2);

    p = strstr(rxData, "R:");
    if (p) roll = atof(p + 2);

    p = strstr(rxData, "Y:");
    if (p) yaw = atof(p + 2);

    Serial.print("Vessel Status : ");
    Serial.println(vesselStatus ? "VESSEL DETECTED" : "DARK VESSEL");

    Serial.print("Pitch : ");
    Serial.println(pitch, 1);

    Serial.print("Roll : ");
    Serial.println(roll, 1);

    Serial.print("Yaw : ");
    Serial.println(yaw, 1);

    Serial.println("--------------------------------");
  }
}
