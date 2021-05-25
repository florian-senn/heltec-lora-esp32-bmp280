/* Heltec Automation I2C scanner example (also it's a basic example how to use I2C1)
 *
 * ESP32 have two I2C (I2C0 and I2C1) bus
 *
 * OLED is connected to I2C0, so if scan with Wire (I2C0), the return address should be 0x3C.
 *
 * If you need scan other device address in I2C1...
 *		- Comment all Wire.***() codes;
 * 		- Uncomment all Wire1.***() codes;
 *
 * I2C scan example and I2C0
 *
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
 * */

#include "Arduino.h"
#include "heltec.h"
#include <Adafruit_BMP280.h>

#define FPS 25
#define MSL_PRESSURE 1019
#define BARO_FACT 10000

Adafruit_BMP280 bmp = Adafruit_BMP280(&Wire1);

uint8_t values[128];
uint8_t valuePointer = 0;

float temperature;
float pressure;
float altitude;
float baro_lo = 2000;
float baro_hi = 0;

u_long now;
u_long refreshSpan = 1000 / FPS;
u_long lastRefresh;
u_long measureSpan = 10000;
u_long lastMeasure;

void setup()
{
  Heltec.begin(true, false, true);
  Wire1.begin(SDA, SCL); //If there have other device on I2C1, scan the device address via I2C1
  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Heltec.display->init();
  Heltec.display->displayOn();
  Heltec.display->invertDisplay();
  Heltec.display->setBrightness(200);
  now = millis();
  lastRefresh = now + refreshSpan;
  lastMeasure = now + measureSpan;
}

void loop()
{
  if (now - lastRefresh > refreshSpan)
  {
    Heltec.display->clear();
    if (now - lastMeasure > measureSpan)
    {
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure() / 100;
      altitude = bmp.readAltitude(MSL_PRESSURE);

      if (pressure < baro_lo)
        baro_lo = pressure;
      if (pressure > baro_hi)
        baro_hi = pressure; 

      values[valuePointer %= 128] = map(pressure * BARO_FACT, baro_lo * BARO_FACT, baro_hi * BARO_FACT, 0, 127);
      valuePointer++;
      lastMeasure = now;
    }
    Heltec.display->drawString(0, 0, "Temperatur: " + String(temperature, 1) + "°C");
    Heltec.display->drawString(0, 10, "Luftdruck: " + String(pressure, 3) + "hPa");
    Heltec.display->drawString(0, 20, "Höhe: " + String(altitude, 2) + "m");
    Heltec.display->drawRect(0, 59, 128, 5);
    Heltec.display->fillRect(0, 60, 128 * (now - lastMeasure) / measureSpan, 4);
    for (uint8_t i = 0; i < 128; i++)
    {
      Heltec.display->drawVerticalLine(i, 32, 25 * values[i] / 127);
    }
    Heltec.display->display();
    lastRefresh = now;
  }
  now = millis();
}