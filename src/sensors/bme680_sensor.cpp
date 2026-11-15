#include "bme680_sensor.h"

#include <Arduino.h>
#include "BME680.h"

static BME680_Class BME680;

bool initBme680(bool heatGas) {
  Serial.println(F("BME680: init"));
  if (!BME680.begin(I2C_STANDARD_MODE)) {
    Serial.println(F("BME680: not available"));
    return false;
  }
  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
  BME680.setIIRFilter(IIR4);
  if (heatGas) {
    BME680.setGas(320, 150);
  } else {
    BME680.setGas(0, 0);
  }
  Serial.printf("BME680: ready, heater=%d\r\n", heatGas ? 1 : 0);
  return true;
}

bool readBme680(int32_t &temperature, int32_t &humidity, int32_t &pressure, int32_t &gas,
                bool heatGas) {
  bool haveReading = false;
  int cycles = heatGas ? BME_CYCLES_FOR_GAS_READINGS : BME_CYCLES_NORMAL;
  gas = 0;
  for (int i = 0; i < cycles; i++) {
    int32_t sampleTemperature = 0;
    int32_t sampleHumidity = 0;
    int32_t samplePressure = 0;
    int32_t sampleGas = 0;
    Serial.printf("BME680: wait %d/%d\r\n", i + 1, cycles);
    BME680.getSensorData(sampleTemperature, sampleHumidity, samplePressure, sampleGas, true);
    Serial.printf("BME680 %d/%d: T=%ld.%02ld C H=%ld.%03ld %% P=%ld.%02ld hPa Gas=%ld ohm\r\n",
                  i + 1, cycles,
                  (long)(sampleTemperature / 100), (long)abs(sampleTemperature % 100),
                  (long)(sampleHumidity / 1000), (long)abs(sampleHumidity % 1000),
                  (long)(samplePressure / 100), (long)(samplePressure % 100), (long)sampleGas);
    // The first sample is not used. The last plausible one is kept.
    if (i == 0) {
      continue;
    }
    bool plausible = sampleTemperature > -2000 && sampleTemperature < 6000 && sampleHumidity > 0 &&
                     sampleHumidity < 100000 && samplePressure >= PRESSURE_MIN_PA &&
                     samplePressure <= PRESSURE_MAX_PA && (!heatGas || sampleGas > 0);
    if (plausible) {
      temperature = sampleTemperature;
      humidity = sampleHumidity;
      pressure = samplePressure;
      gas = heatGas ? sampleGas : 0;
      haveReading = true;
    }
  }
  if (!haveReading) {
    Serial.println(F("BME680: no valid reading"));
  }
  return haveReading;
}
