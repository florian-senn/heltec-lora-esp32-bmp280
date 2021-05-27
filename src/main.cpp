#include "Arduino.h"
#include "heltec.h"
#include <Adafruit_BMP280.h>

#define FPS 20
#define MSL_PRESSURE 1019
#define BARO_FACT 10000

Adafruit_BMP280 bmp = Adafruit_BMP280(&Wire1);

float values[128];
uint8_t valuePointer = 0;

float temperature;
float pressure;
float altitude;
float baro_lo = 2000;
float baro_hi = 0;

u_long now;
u_long refreshSpan = 1000 / FPS;
u_long lastRefresh;
u_long measureSpan = 2500;
u_long lastMeasure;

void setup()
{
  Heltec.begin(true, false, true);
  Wire1.begin(SDA, SCL);
  bmp.begin(BMP280_ADDRESS_ALT);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X4,     /* Temp. oversampling */
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

      values[valuePointer %= 127] = pressure;
      valuePointer++;
      lastMeasure = now;
    }
    Heltec.display->drawString(0, 0, String(pressure, 4) + "hPa, " + String(altitude, 2) + "m");
    Heltec.display->drawRect(0, 61, 128, 3);
    Heltec.display->fillRect(0, 62, 128 * (now - lastMeasure) / measureSpan, 1);
    Heltec.display->drawString(0, 10, String(baro_hi, 4));
    Heltec.display->drawString(0, 49, String(baro_lo, 4));
    for (uint8_t i = 0; i < 128; i++)
    {
      //float value = values[i];
      //uint8_t temp = ((baro_hi - value) / (baro_hi - baro_lo)) * 127;
    }
    Heltec.display->display();
    lastRefresh = now;
  }
  now = millis();
}