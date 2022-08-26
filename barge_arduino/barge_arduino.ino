#include <SPI.h>              // For LoRa radio
#include <LoRa.h>             // For LoRa RFM95C
#include <CRC8.h>             // For LoRa radio - TODO - See if this is necessary
#include <CRC.h>              // For LoRa radio - TODO - See if this is necessary
#include <Adafruit_Sensor.h>    // For the BMP180
#include <Adafruit_BMP085_U.h>  // For the BMP180
#include <Ewma.h>   // Sensor data smoothing - Exponentially Weighted Moving Average
#include <YetAnotherPcInt.h>  // Easy use of PcInt
#include "LowPower.h"         // RocketScream low-power for AVR

#include "AK09918.h"
#include "ICM20600.h"

const int16_t lora_csPin = 5;         // LoRa radio chip select
const int16_t lora_resetPin = 6;       // LoRa radio reset

CRC8 crc;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10077);

const int32_t read_interval = 5010;    // interval between reads (ms)
const int32_t send_interval = read_interval * 6;   // interval between sends (ms)
// Suggest change to 301681 milliseconds (Just over than 5 minutes. An odd number to reduce the chance of repeated collisions)

uint32_t  lastReadTime = 0;        // last time data was measured
uint32_t  lastSendTime = 0;        // last time data was sent

const float filterAlpha = 0.05;     // Smaller is more smoothing
Ewma pressureFilter(filterAlpha);
Ewma temperatureFilter(filterAlpha);
Ewma windFilter(float(read_interval) / 20000);   // For 2 minute wind average at read_interval of 5s
Ewma gustFilter(float(read_interval) / 12000);   // For 15 second gusts at read_interval of 5s
float max_gust = 0;
const uint8_t wind_irqPin = A9;        // Wind sensor irq pin (d9 - adc12 - a9 - pcint5)
volatile uint16_t  wind_rotation_count = 0;

AK09918 ak09918;
AK09918_err_type_t err;
// Lab tested: Min_X: -59, Max_X: 61, Min_Y: -91, Max_Y: 38, Min_Z: -93, Max_Z: 35
int32_t offset_x = 1;
int32_t offset_y = -26;
int32_t offset_z = -29;

ICM20600 icm20600(true);
int32_t x, y, z;
uint16_t acc_x_max = 0;
uint16_t acc_y_max = 0;
uint16_t acc_z_max = 0;

void setup() {
  //Serial.begin(9600);                   // initialize serial
  //while (!Serial);
  //Serial.println("GSC Barge Wind Sensor");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(lora_csPin, lora_resetPin); // set CS, reset, no IRQ
  LoRa.setSPIFrequency(1E6);

  if (!LoRa.begin(906.2E6)) {             // initialize ratio at 915 MHz
    //Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.setSpreadingFactor(12); // Larger spreading factors give more range, 6-12
  LoRa.disableCrc();
  LoRa.setTxPower(20); // 2-20
  LoRa.onTxDone(onLoRaTxDone);

  //Serial.println("LoRa init succeeded.");

  //Serial.print("Filter alpha: "); Serial.println(filterAlpha);
  //Serial.print("Interval: "); Serial.print(read_interval); Serial.print(" : "); Serial.println(send_interval);

  if (filterAlpha >= 1 && filterAlpha <= 0) {
    //Serial.println("Alpha for Ewma must be between zero and 1 not inclusive");
    while(1);
  }

  crc.setPolynome(0x07);

  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    //Serial.print("BMP180 not detected ... Check your wiring or I2C address!");
    while(1);
  }
  //Serial.println("BMP180 init succeeded.");

  // Setup Compass
  err = ak09918.initialize();
  ak09918.switchMode(AK09918_POWER_DOWN);
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
  err = ak09918.isDataReady();
  while (err != AK09918_ERR_OK) {
      //Serial.println("Waiting Sensor");
      delay(100);
      err = ak09918.isDataReady();
  }
  //Serial.println("icm20600 init succeeded.");
  // Calibrated at lab bench. May need to be revisited.
  //calibrate(30000, &offset_x, &offset_y, &offset_z);

  // Setup accelerometer and gyro
  icm20600.initialize();
  //Serial.println("icm20600 init succeeded.");

  // Setup Wind Sensor
  pinMode(A0, INPUT); // Messed up the wiring
  pinMode(A9, INPUT_PULLUP);

  PcInt::attachInterrupt(A9, wind_rotation_count_irq, CHANGE);
}

void onLoRaTxDone() {
  LoRa.sleep();                         // Put radio into lowest power mode
  lowPowerMode();
}

void wind_rotation_count_irq() {
  wind_rotation_count += 1;
  lowPowerMode();
}

void loop() {
  uint32_t currentTime = millis();
  uint32_t readTimeInterval = currentTime - lastReadTime; // Overflow is okay for intervals

  if (readTimeInterval > read_interval) {
    lastReadTime = millis();

    // Wind should be first to be closest to reading of the currentTime.
    cli(); // Disable interrupts
    uint16_t  wind_count = wind_rotation_count;
    wind_rotation_count = 0;
    sei(); // Enable interrupts
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

    ak09918.getData(&x, &y, &z);
    x = x - offset_x;
    y = y - offset_y;
    z = z - offset_z;
    //Serial.print("M:  "); Serial.print(x);  Serial.print(",  ");
    //Serial.print(y);  Serial.print(",  "); Serial.print(z);  Serial.println(" uT");

    // get acceleration
    uint16_t acc_x = abs(icm20600.getAccelerationX());
    uint16_t acc_y = abs(icm20600.getAccelerationY());
    uint16_t acc_z = abs(icm20600.getAccelerationZ());
    if (acc_x_max < acc_x) { acc_x_max = acc_x; }
    if (acc_y_max < acc_y) { acc_y_max = acc_y; }
    if (acc_z_max < acc_z) { acc_z_max = acc_z; }

    //Serial.print("A:  "); Serial.print(acc_x_max); Serial.print(",  ");
    //Serial.print(acc_y_max); Serial.print(",  "); Serial.print(acc_z_max); Serial.println(" mg");

    delay(100); // Wait for the BMP180 to fully wake up
    float rawTemperature;
    bmp.getTemperature(&rawTemperature);
    if (temperatureFilter.output == 0 || abs(temperatureFilter.output - rawTemperature) < 10) {
      // Ignore if the temperature change was more than 10 degrees C
      // The chip or library gives spurious readings occationally
      temperatureFilter.filter(rawTemperature);
    }
    //Serial.print("temperature = "); Serial.print(temperatureFilter.output); Serial.print(" raw: "); Serial.print(rawTemperature);  Serial.println(" C");

    float rawPressure;
    bmp.getPressure(&rawPressure);
    if (pressureFilter.output == 0 || abs(pressureFilter.output - rawPressure) < 4) {
      // Ignore if the pressure change was more than 4mbar (equivalent to 30m altitude change)
      // The chip or library gives spurious readings occationally
      float pressure = pressureFilter.filter(rawPressure);
    }
    //Serial.print("pressure = "); Serial.print(pressure); Serial.print(" raw: "); Serial.print(rawPressure);  Serial.println(" Pa");

    //Serial.print("pressure = "); Serial.print(pressureFilter.output); Serial.println(" Pa");
    //Serial.print("temperature = "); Serial.print(temperatureFilter.output); Serial.println(" C");
  }

  uint32_t  sendTimeInterval = currentTime - lastSendTime;  // Overflow is okay for intervals
  if (sendTimeInterval > send_interval) {
    lastSendTime = millis();

    // Battery sensor
    int16_t battery_level = analogRead(A1);  // read the battery voltage divider
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    sendGSCData(temperatureFilter.output,
        pressureFilter.output,
        windFilter.output,
        max_gust,
        (int16_t)x,
        (int16_t)y,
        acc_z_max,
        battery_level);
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

    max_gust = 0;
    acc_x_max = acc_y_max = acc_z_max = 0;
  }

  lowPowerMode();
}

void lowPowerMode() {
  LowPower.idle(SLEEP_1S, ADC_ON,
                TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_ON,
                SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
}

void packUnsignedShort(char* buffer, uint16_t  value) {
  buffer[0] = highByte(value);
  buffer[1] = lowByte(value);
}

void packSignedShort(char* buffer, int16_t value) {
  buffer[0] = highByte(value);
  buffer[1] = lowByte(value);
}

void sendGSCData(float temperature,
                 float pressure,
                 float wind,
                 float gust,
                 int16_t x,
                 int16_t y,
                 uint16_t acc_z,
                 int16_t battery) {
  //Serial.println("sendGSCData");
  LoRa.idle(); // Wake up the radio before prepping the packet to get it ready to send

  int16_t packet_length = 4 + 20 + 1; // header, payload, crc
  char buffer[packet_length];
  memset(buffer, 1, packet_length);
  buffer[0] = packet_length - 1; // Length of of message minus crc
  String header = "GSC ";
  header.toCharArray(&(buffer[1]), header.length());

  int16_t temperature_int = (uint16_t)(temperature * 100);
  //Serial.print("temperature = "); Serial.print(temperature_int); Serial.println(" c°C");
  packSignedShort(&buffer[4], temperature_int);

  uint16_t  pressure_offset_int = (uint16_t)(pressure - 80000);
  packUnsignedShort(&buffer[6], pressure_offset_int);
  //Serial.print("pressure = "); Serial.print(pressureFilter.output); Serial.print("Pa, "); Serial.println(pressure_offset_int);

  uint16_t  wind_int = (uint16_t)(wind * 100);
  //Serial.print("wind revs = "); Serial.print(wind_int); Serial.println(" ");
  packUnsignedShort(&buffer[8], wind_int);

  uint16_t  gust_int = (uint16_t)(gust * 100);
  //Serial.print("gust revs = "); Serial.print(gust_int); Serial.println(" ");
  packUnsignedShort(&buffer[10], gust_int);

  //Serial.print("Compass X = "); Serial.print(x); Serial.println(" ");
  packSignedShort(&buffer[12], x);
  packSignedShort(&buffer[14], y);
  packUnsignedShort(&buffer[16], acc_z);
  packSignedShort(&buffer[18], battery);

  crc.restart();
  crc.add(buffer, packet_length-1);
  buffer[packet_length-1] = crc.getCRC();

  LoRa.beginPacket();                   // start packet
  LoRa.write(buffer, packet_length);    // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void calibrate(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz) {
    int32_t value_x_min = 0;
    int32_t value_x_max = 0;
    int32_t value_y_min = 0;
    int32_t value_y_max = 0;
    int32_t value_z_min = 0;
    int32_t value_z_max = 0;
    uint32_t timeStart = 0;

    int32_t x, y, z;
    ak09918.getData(&x, &y, &z);

    value_x_min = x;
    value_x_max = x;
    value_y_min = y;
    value_y_max = y;
    value_z_min = z;
    value_z_max = z;
    delay(100);
    Serial.println("Calculating offset :  ");

    timeStart = millis();

    while ((millis() - timeStart) < timeout) {
        ak09918.getData(&x, &y, &z);

        /* Update x-Axis max/min value */
        if (value_x_min > x) {
            value_x_min = x;
            // Serial.print("Update value_x_min: ");
            // Serial.println(value_x_min);

        } else if (value_x_max < x) {
            value_x_max = x;
            // Serial.print("update value_x_max: ");
            // Serial.println(value_x_max);
        }

        /* Update y-Axis max/min value */
        if (value_y_min > y) {
            value_y_min = y;
            // Serial.print("Update value_y_min: ");
            // Serial.println(value_y_min);

        } else if (value_y_max < y) {
            value_y_max = y;
            // Serial.print("update value_y_max: ");
            // Serial.println(value_y_max);
        }

        /* Update z-Axis max/min value */
        if (value_z_min > z) {
            value_z_min = z;
            // Serial.print("Update value_z_min: ");
            // Serial.println(value_z_min);

        } else if (value_z_max < z) {
            value_z_max = z;
            // Serial.print("update value_z_max: ");
            // Serial.println(value_z_max);
        }
        delay(100);

    }
    *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
    *offsety = value_y_min + (value_y_max - value_y_min) / 2;
    *offsetz = value_z_min + (value_z_max - value_z_min) / 2;

    Serial.print("Min :  ");
    Serial.print(value_x_min);
    Serial.print(",  ");
    Serial.print(value_y_min);
    Serial.print(",  ");
    Serial.print(value_z_min);
    Serial.print(" uT");
    Serial.print("    Max :  ");
    Serial.print(value_x_max);
    Serial.print(",  ");
    Serial.print(value_y_max);
    Serial.print(",  ");
    Serial.print(value_z_max);
    Serial.println(" uT");
}
