#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <CRC8.h>
#include <CRC.h>
#include <Adafruit_MPL3115A2.h>


const int csPin = 10;         // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

CRC8 crc;
Adafruit_MPL3115A2 baro;

long lastSendTime = 0;        // last send time
int send_interval = 5000;    // interval between sends
// Change to 301681 milliseconds (Just over than 5 minutes. An odd number to reduce the chance of repeated collisions)

void setup() {
  Serial.begin(57600);                   // initialize serial
  while (!Serial);

  Serial.println("GSC Barge Wind Sensor");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");

  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }
  baro.setSeaPressure(1013.26);
  Serial.println("MPL3115A2 init succeeded.");

  crc.setPolynome(0x07);
}

void loop() {
  if (millis() < lastSendTime) {
    // Avoid clock overflow
    lastSendTime = millis() - send_interval - 1;
  }

  float pressure = baro.getPressure();
  unsigned short pressure_int = (unsigned short)((pressure - 80000) * 1000);
  float temperature = baro.getTemperature();
  short temperature_int = (short)(temperature * 100);

  if (millis() - lastSendTime > send_interval) {
    Serial.print("pressure = "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("pressure = "); Serial.print(pressure_int); Serial.println(" Pa");
    Serial.print("temperature = "); Serial.print(temperature); Serial.println(" C");
    Serial.print("temperature = "); Serial.print(temperature_int); Serial.println(" c°C");

    sendGSCData(temperature_int, pressure_int);

    lastSendTime = millis();            // timestamp the message
  }

  delay(250);
}

void packUnsignedShort(char* buffer, unsigned short value) {
  buffer[0] = highByte(value);
  buffer[1] = lowByte(value);
}

void packSignedShort(char* buffer, short value) {
  buffer[0] = highByte(value);
  buffer[1] = lowByte(value);
}

void sendGSCData(short temperature, unsigned short pressure) {
  Serial.println("sendGSCData");
  int packet_length = 4 + 4 + 1; // header, payload, crc
  char buffer[packet_length];
  memset(buffer, 1, packet_length);
  buffer[0] = packet_length - 1; // Length of of message minus crc
  String header = "GSC ";
  header.toCharArray(&(buffer[1]), header.length());
  packSignedShort(&buffer[4], temperature);
  packUnsignedShort(&buffer[6], pressure);


  crc.restart();
  crc.add(buffer, packet_length-1);
  buffer[packet_length-1] = crc.getCRC();

  LoRa.beginPacket();                   // start packet
  LoRa.write(buffer, packet_length); // add payload
  LoRa.endPacket();                     // finish packet and send it
}
