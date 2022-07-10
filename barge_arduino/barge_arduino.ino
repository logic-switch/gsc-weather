#include <SPI.h>              // For LoRa radio
#include <LoRa.h>             // For LoRa RFM95C
#include <CRC8.h>             // For LoRa radio - TODO - See if this is necessary
#include <CRC.h>              // For LoRa radio - TODO - See if this is necessary
#include <Adafruit_Sensor.h>    // For the BMP180
#include <Adafruit_BMP085_U.h>  // For the BMP180
#include <Ewma.h>   // Sensor data smoothing - Exponentially Weighted Moving Average
#include <limits.h> // For ULONG_MAX

const int lora_csPin = 10;         // LoRa radio chip select
const int lora_resetPin = 9;       // LoRa radio reset
const int lora_irqPin = 2;         // LoRa irq pin - must be a hardware interrupt pin
const int wind_irqPin = 3;         // Wind sensor irq pin - must be a hardware interrupt pin
volatile unsigned int wind_rotation_count = 0;

CRC8 crc;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10077);

const long read_interval = 5000;    // interval between reads (ms)
const long send_interval = 120010;    // interval between sends (ms)
// Suggest change to 301681 milliseconds (Just over than 5 minutes. An odd number to reduce the chance of repeated collisions)

unsigned long lastReadTime = 0;        // last time data was measured
unsigned long lastSendTime = 0;        // last time data was sent

const float filterAlpha = 0.05;     // Smaller is more smoothing
Ewma pressureFilter(filterAlpha);
Ewma temperatureFilter(filterAlpha);
Ewma windFilter(float(read_interval) / 25000);   // For 2 minute wind average at read_interval of 5s
Ewma gustFilter(float(read_interval) / 8000);   // For 15 second gusts at read_interval of 5s
float max_gust = 0;

void setup() {
  Serial.begin(57600);                   // initialize serial
  while (!Serial);

  Serial.println("GSC Barge Wind Sensor");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(lora_csPin, lora_resetPin, lora_irqPin);// set CS, reset, IRQ pin
  LoRa.setSPIFrequency(1E6);

  if (!LoRa.begin(906.2E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.setPreambleLength(16);
  LoRa.setSpreadingFactor(12); // Larger spreading factors give more range, 6-12
  LoRa.disableCrc();
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setTxPower(20); // 2-20

  Serial.println("LoRa init succeeded.");

  Serial.print("Filter alpha: "); Serial.println(filterAlpha);
  Serial.print("Interval: "); Serial.print(read_interval); Serial.print(" : "); Serial.println(send_interval);

  if (filterAlpha >= 1 && filterAlpha <= 0) {
    Serial.println("Alpha for Ewma must be between zero and 1 not inclusive");
    while(1);
  }

  crc.setPolynome(0x07);

  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.print("BMP180 not detected ... Check your wiring or I2C address!");
    while(1);
  }

  // Setup Wind Sensor
  attachInterrupt(digitalPinToInterrupt(wind_irqPin), wind_rotation_count_irq, CHANGE);
}

void wind_rotation_count_irq() {
  wind_rotation_count += 1;
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long readTimeInterval = currentTime - lastReadTime;
  if (currentTime < lastReadTime) {
    // Avoid clock overflow
    readTimeInterval = ULONG_MAX - lastReadTime + currentTime;
  }

  if (readTimeInterval > read_interval) {
    lastReadTime = millis();

    // Wind should be first to be closest to reading of the currentTime.
    unsigned int wind_count = wind_rotation_count;
    wind_rotation_count = 0;
    float rps = float(wind_count) * 1000 / readTimeInterval;
    float wind = windFilter.filter(rps);
    float gust = gustFilter.filter(rps);
    if (gust < wind) {
      gust = wind;
    }
    if (max_gust < gust) {
      max_gust = gust;
    }
    //Serial.print("  RPS  = "); Serial.print(rps); Serial.print("  Wind = "); Serial.print(wind);
    //Serial.print("  Gust = "); Serial.print(gust); Serial.print("  Max = "); Serial.print(max_gust); Serial.println("");

    float rawTemperature;
    bmp.getTemperature(&rawTemperature);
    float temperature = temperatureFilter.filter(rawTemperature);
    //Serial.print("temperature = "); Serial.print(temperature); Serial.print(" raw: "); Serial.print(rawTemperature);  Serial.println(" C");

    float rawPressure;
    bmp.getPressure(&rawPressure);
    float pressure = pressureFilter.filter(rawPressure);
    //Serial.print("pressure = "); Serial.print(pressure); Serial.print(" raw: "); Serial.print(rawPressure);  Serial.println(" Pa");

    //Serial.print("pressure = "); Serial.print(pressureFilter.output); Serial.println(" Pa");
    //Serial.print("temperature = "); Serial.print(temperatureFilter.output); Serial.println(" C");
  }

  unsigned long sendTimeInterval = currentTime - lastSendTime;
  if (currentTime < lastSendTime) {
    // Avoid clock overflow
    sendTimeInterval = ULONG_MAX - lastSendTime + currentTime;
  }
  if (sendTimeInterval > send_interval) {
    lastSendTime = millis();

    unsigned short pressure_offset_int = (unsigned short)((pressureFilter.output - 80000));
    short temperature_int = (short)(temperatureFilter.output * 100);

    Serial.print("pressure = "); Serial.print(pressureFilter.output); Serial.print("Pa, "); Serial.println(pressure_offset_int);
    Serial.print("temperature = "); Serial.print(temperature_int); Serial.println(" c°C");
    sendGSCData(temperature_int, pressure_offset_int);

    max_gust = 0;
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

void sendGSCData(short s1, unsigned short s2) {
  Serial.println("sendGSCData");
  int packet_length = 4 + 4 + 1; // header, payload, crc
  char buffer[packet_length];
  memset(buffer, 1, packet_length);
  buffer[0] = packet_length - 1; // Length of of message minus crc
  String header = "GSC ";
  header.toCharArray(&(buffer[1]), header.length());
  packSignedShort(&buffer[4], s1);
  packUnsignedShort(&buffer[6], s2);


  crc.restart();
  crc.add(buffer, packet_length-1);
  buffer[packet_length-1] = crc.getCRC();

  LoRa.beginPacket();                   // start packet
  LoRa.write(buffer, packet_length); // add payload
  LoRa.endPacket();                     // finish packet and send it
}
