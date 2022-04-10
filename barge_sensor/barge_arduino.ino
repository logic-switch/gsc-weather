#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <CRC8.h>
#include <CRC.h>

const int csPin = 10;         // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

CRC8 crc;

long lastSendTime = 0;        // last send time
int interval = 10000;         // interval between sends
// Change to 301681 milliseconds (Just over than 5 minutes. An odd number to reduce the chance of repeated collisions)

void setup() {
  Serial.begin(57000);                   // initialize serial
  while (!Serial);

  Serial.println("GSC Barge Wind Sensor");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");

  crc.setPolynome(0x07);
}

void loop() {
  if (millis() < lastSendTime) {
    // Avoid clock overflow
    lastSendTime = millis() - interval - 1;
  }
  if (millis() - lastSendTime > interval) {
    String message = "GSC";   // send a message
    Serial.println("Sending:" + message);
    Serial.println(message.length());
    sendMessage(message);
    lastSendTime = millis();            // timestamp the message
  }
}

void sendMessage(String outgoing) {
  Serial.println(outgoing.length());

  char buffer[outgoing.length() + 2];
  buffer[0] = outgoing.length() + 2; // Length of overall message
  outgoing.toCharArray(&(buffer[1]), outgoing.length());

  crc.restart();
  crc.add(buffer, outgoing.length()+1);
  buffer[outgoing.length()+1] = crc.getCRC();

  LoRa.beginPacket();                   // start packet
  LoRa.write(buffer, outgoing.length()+2); // add payload
  LoRa.endPacket();                     // finish packet and send it
}