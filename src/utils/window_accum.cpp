#include "window_accum.h"

#include <string.h>

void WindowAccum::reset() {
  memset(this, 0, sizeof(*this));
  humidityScoreMin = 256;
}

void WindowAccum::addBatterySample(unsigned int batteryMvValue) {
  batteryMv = batteryMvValue;
  if (sampleCount < 255) {
    sampleCount++;
  }
}

void WindowAccum::addTemperature1Sample(int32_t temperature1) {
  temp1Sum += temperature1;
  temp1Count++;
}

void WindowAccum::addTemperature2Sample(int32_t temperature2) {
  temp2Sum += temperature2;
  temp2Count++;
}

void WindowAccum::addPressureSample(int32_t pressure) {
  pressureSum += pressure;
  pressureCount++;
}

void WindowAccum::addHumiditySample(int32_t humidity1Value, int32_t humidity2Value,
                                    int humidityScore) {
  if (humidityScore >= humidityScoreMin) {
    return;
  }
  humidityScoreMin = humidityScore;
  humidity1 = humidity1Value;
  humidity2 = humidity2Value;
  hasHumidity = true;
}

void WindowAccum::addCo2Sample(int32_t co2) {
  if (!hasCo2 || co2 > co2Max) {
    co2Max = co2;
  }
  hasCo2 = true;
}

void WindowAccum::addGasSample(int32_t gas) {
  if (gas <= 0) {
    return;
  }
  if (!hasGas || gas < gasMin) {
    gasMin = gas;
  }
  hasGas = true;
}

void WindowAccum::addAqiSample(int airQualityScore) {
  if (!hasAqi || airQualityScore < aqiMin) {
    aqiMin = airQualityScore;
  }
  hasAqi = true;
}

int32_t WindowAccum::meanTemp1() const {
  return temp1Count ? temp1Sum / temp1Count : 0;
}

int32_t WindowAccum::meanTemp2() const {
  return temp2Count ? temp2Sum / temp2Count : 0;
}

int32_t WindowAccum::meanPressure() const {
  return pressureCount ? pressureSum / pressureCount : 0;
}

int32_t WindowAccum::packedHumidity1() const {
  return hasHumidity ? humidity1 : 0;
}

int32_t WindowAccum::packedHumidity2() const {
  return hasHumidity ? humidity2 : 0;
}

int32_t WindowAccum::packedGas() const {
  return hasGas ? gasMin : 0;
}

int32_t WindowAccum::packedCo2() const {
  return hasCo2 ? co2Max : 0;
}

int WindowAccum::packedAqi() const {
  return hasAqi ? aqiMin : 0;
}
  