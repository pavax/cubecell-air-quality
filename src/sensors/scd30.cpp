#include "scd30.h"

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"

static SCD30 airSensor;

uint16_t toPressureMbar(int32_t pressurePa) {
  int32_t pressureMbar = pressurePa / 100;
  if (pressureMbar < SCD30_PRESSURE_MIN_MBAR) {
    return SCD30_PRESSURE_MIN_MBAR;
  }
  if (pressureMbar > SCD30_PRESSURE_MAX_MBAR) {
    return SCD30_PRESSURE_MAX_MBAR;
  }
  return pressureMbar;
}

bool attachScd30() {
  if (!airSensor.begin(Wire, false, false)) {
    return false;
  }
  airSensor.setAutoSelfCalibration(false);
  return true;
}

bool initScd30(uint16_t pressureMbar) {
  Serial.printf("SCD30: connect, pressure=%u mbar\r\n", pressureMbar);
  if (!attachScd30()) {
    Wire.beginTransmission(SCD30_ADDRESS);
    Wire.write(highByte(COMMAND_READ_FW_VER));
    Wire.write(lowByte(COMMAND_READ_FW_VER));
    uint8_t err = Wire.endTransmission();
    Serial.printf("SCD30: not available, fw err=%u\r\n", err);
    return false;
  }

  airSensor.StopMeasurement();
  delay(500);
  Serial.printf("SCD30: interval %d s\r\n", SCD30_MEASUREMENT_INTERVAL);
  airSensor.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL);
  bool started = pressureMbar == 0 ? airSensor.beginMeasuring() : airSensor.beginMeasuring(pressureMbar);
  if (!started) {
    Serial.println(F("SCD30: start failed"));
    return false;
  }
  Serial.println(F("SCD30: measuring"));
  return true;
}

bool setScd30AmbientPressure(uint16_t pressureMbar) {
  if (pressureMbar == 0) {
    return airSensor.setAmbientPressure(0);
  }
  if (pressureMbar < SCD30_PRESSURE_MIN_MBAR || pressureMbar > SCD30_PRESSURE_MAX_MBAR) {
    Serial.printf("SCD30: pressure out of range (%u-%u mbar)\r\n",
                  SCD30_PRESSURE_MIN_MBAR, SCD30_PRESSURE_MAX_MBAR);
    return false;
  }
  return airSensor.setAmbientPressure(pressureMbar);
}

bool getScd30ForcedRecalibration(uint16_t &ppm) {
  return airSensor.getForcedRecalibration(&ppm);
}

void printScd30Settings(uint16_t ambientPressureMbar) {
  uint16_t val = 0;
  Serial.println(F("--- SCD30 settings ---"));
  if (airSensor.getFirmwareVersion(&val)) {
    Serial.printf("Firmware=0x%04X\r\n", val);
  } else {
    Serial.println(F("Firmware=read failed"));
  }
  if (airSensor.getForcedRecalibration(&val)) {
    Serial.printf("FRC factor=%u ppm\r\n", val);
  } else {
    Serial.println(F("FRC factor=read failed"));
  }
  if (airSensor.getMeasurementInterval(&val)) {
    Serial.printf("Interval=%u s\r\n", val);
  } else {
    Serial.println(F("Interval=read failed"));
  }
  Serial.printf("ASC=%s\r\n", airSensor.getAutoSelfCalibration() ? "on" : "off");
  Serial.printf("Temp offset=%.2f C\r\n", airSensor.getTemperatureOffset());
  if (airSensor.getAltitudeCompensation(&val)) {
    Serial.printf("Altitude=%u m\r\n", val);
  } else {
    Serial.println(F("Altitude=read failed"));
  }
  Serial.printf("Ambient pressure cmd=%u mbar\r\n", ambientPressureMbar);
  Serial.println(F("----------------------"));
}

bool pollScd30(uint16_t &co2, float &temperature, float &humidity) {
  if (!airSensor.dataAvailable()) {
    return false;
  }
  co2 = airSensor.getCO2();
  temperature = airSensor.getTemperature();
  humidity = airSensor.getHumidity();
  return true;
}

bool calibrateScd30(uint16_t refPpm) {
  Serial.printf("SCD30: calibrating for %d s in fresh air (ref=%u ppm)\r\n",
                SCD30_CALIBRATION_TIME_S, refPpm);

  uint16_t frcBefore = 0;
  if (airSensor.getForcedRecalibration(&frcBefore)) {
    Serial.printf("SCD30: FRC before=%u ppm\r\n", frcBefore);
  } else {
    Serial.println(F("SCD30: FRC before=read failed"));
  }

  const unsigned long calibMs = (unsigned long)SCD30_CALIBRATION_TIME_S * 1000UL;
  unsigned long start = millis();
  while ((millis() - start) < calibMs) {
    if (airSensor.dataAvailable()) {
      int32_t sampleCo2 = airSensor.getCO2();
      int32_t sampleTemperature = airSensor.getTemperature() * 100;
      int32_t sampleHumidity = airSensor.getHumidity() * 1000;
      Serial.printf("SCD30: t=%lu s CO2=%ld ppm T=%ld.%02ld C H=%ld.%03ld %%\r\n",
                    (unsigned long)((millis() - start) / 1000UL), (long)sampleCo2,
                    (long)(sampleTemperature / 100), (long)abs(sampleTemperature % 100),
                    (long)(sampleHumidity / 1000), (long)abs(sampleHumidity % 1000));
    }
    delay(SCD30_POLL_MS);
  }

  unsigned long beforeStart = millis();
  bool gotBefore = false;
  while ((millis() - beforeStart) < SCD30_READ_TIMEOUT_MS) {
    if (airSensor.dataAvailable()) {
      Serial.printf("SCD30: before FRC CO2=%u ppm\r\n", airSensor.getCO2());
      gotBefore = true;
      break;
    }
    delay(SCD30_POLL_MS);
  }
  if (!gotBefore) {
    Serial.println(F("SCD30: before FRC sample timed out"));
  }

  bool setOk = airSensor.setForcedRecalibrationFactor(refPpm);
  airSensor.setAutoSelfCalibration(false);
  Serial.printf("SCD30: setForcedRecalibrationFactor(%u)=%s\r\n",
                refPpm, setOk ? "ok" : "FAIL");

  uint16_t frcAfter = 0;
  if (airSensor.getForcedRecalibration(&frcAfter)) {
    Serial.printf("SCD30: FRC after=%u ppm (expected %u)\r\n", frcAfter, refPpm);
  } else {
    Serial.println(F("SCD30: FRC after=read failed"));
  }

  return setOk;
}

uint8_t readScd30(int32_t &co2, int32_t &temperature, int32_t &humidity, bool discardFirst) {
  unsigned long start = millis();
  bool discardedFirst = !discardFirst;
  while ((millis() - start) < SCD30_READ_TIMEOUT_MS) {
    if (airSensor.dataAvailable()) {
      int32_t sampleCo2 = airSensor.getCO2();
      int32_t sampleTemperature = airSensor.getTemperature() * 100;
      int32_t sampleHumidity = airSensor.getHumidity() * 1000;
      Serial.printf("SCD30: CO2=%ld ppm T=%ld.%02ld C H=%ld.%03ld %%\r\n",
                    (long)sampleCo2, (long)(sampleTemperature / 100), (long)abs(sampleTemperature % 100),
                    (long)(sampleHumidity / 1000), (long)abs(sampleHumidity % 1000));
      // The first sample after a fresh start has not settled.
      if (!discardedFirst) {
        discardedFirst = true;
        Serial.println(F("SCD30: discard first sample"));
        delay(SCD30_POLL_MS);
        continue;
      }
      co2 = sampleCo2;
      temperature = sampleTemperature;
      humidity = sampleHumidity;
      return 1;
    }
    Serial.printf("SCD30: waiting %lu ms\r\n", millis() - start);
    delay(SCD30_POLL_MS);
  }
  Serial.println(F("SCD30: timed out"));
  return 9;
}
