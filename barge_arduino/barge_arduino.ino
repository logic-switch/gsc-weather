#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <CRC8.h>
#include <CRC.h>
#include <Adafruit_MPL3115A2.h>
#include <Ewma.h>


const int csPin = 10;         // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

CRC8 crc;
Adafruit_MPL3115A2 baro;

const long read_interval = 1000;    // interval between sends (ms)
const long send_interval = 5010;    // interval between sends (ms)
// Change to 301681 milliseconds (Just over than 5 minutes. An odd number to reduce the chance of repeated collisions)

long lastReadTime = -1 * read_interval;        // last time data was measured
long lastSendTime = -1 * send_interval;        // last time data was sent

const float filterAlpha = 0.1;     // Smaller is more smoothing
Ewma pressureFilter(filterAlpha);
Ewma temperatureFilter(filterAlpha);

void setup() {
  Serial.begin(57600);                   // initialize serial
  while (!Serial);

  Serial.println("GSC Barge Wind Sensor");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  LoRa.setSPIFrequency(1E6);

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.setPreambleLength(16);
  LoRa.setSpreadingFactor(12); // Larger spreading factors give more range, 6-12
  LoRa.disableCrc();

  Serial.println("LoRa init succeeded.");

  Serial.print("Filter alpha: "); Serial.println(filterAlpha);
  Serial.print("Interval: "); Serial.print(read_interval); Serial.print(" : "); Serial.println(send_interval);

  if (filterAlpha >= 1 && filterAlpha <= 0) {
    Serial.println("Alpha for Ewma must be between zero and 1 not inclusive");
    while(1);
  }

  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }
  baro.setSeaPressure(1013.26);
  Serial.println("MPL3115A2 init succeeded.");

  crc.setPolynome(0x07);
}

void loop() {
  long currentTime = millis();
  if (currentTime < lastSendTime) {
    // Avoid clock overflow
    lastSendTime = currentTime - send_interval - 1;
  }
  if (currentTime < lastReadTime) {
    // Avoid clock overflow
    lastReadTime = currentTime - read_interval - 1;
  }

  if (currentTime - lastReadTime > read_interval) {
    float rawTemperature = baro.getTemperature();
    float temperature = temperatureFilter.filter(rawTemperature);
    Serial.print("temperature = "); Serial.print(temperature); Serial.print(" raw: "); Serial.print(rawTemperature);  Serial.println(" C");


    float pressure = pressureFilter.filter(baro.getPressure());
    //float temperature = temperatureFilter.filter(baro.getTemperature());

    //Serial.print("pressure = "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("temperature = "); Serial.print(temperature); Serial.println(" C");

    lastReadTime = millis();
  }

  if (currentTime - lastSendTime > send_interval) {
    unsigned short pressure_int = (unsigned short)((pressureFilter.output - 800) * 100);
    short temperature_int = (short)(temperatureFilter.output * 100);

    Serial.print("pressure = "); Serial.print(pressureFilter.output); Serial.print("hPa, "); Serial.print(pressure_int); Serial.println(" Pa");
    Serial.print("temperature = "); Serial.print(temperature_int); Serial.println(" c°C");
    sendGSCData(temperature_int, pressure_int);

    lastSendTime = millis();
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
