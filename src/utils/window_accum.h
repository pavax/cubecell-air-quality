#ifndef WINDOW_ACCUM_H
#define WINDOW_ACCUM_H

#include <stdint.h>

// Aggregates samples between uplinks (mean / max / min / worst humidity score).
struct WindowAccum {
  int32_t temp1Sum;
  int32_t temp2Sum;
  uint16_t temp1Count;
  uint16_t temp2Count;
  int32_t pressureSum;
  uint16_t pressureCount;
  int32_t humidity1;
  int32_t humidity2;
  int humidityScoreMin;
  bool hasHumidity;
  int32_t co2Max;
  bool hasCo2;
  int32_t gasMin;
  bool hasGas;
  int aqiMin;
  bool hasAqi;
  unsigned int batteryMv;

  void reset();
  void addBatterySample(unsigned int batteryMvValue);
  void addTemperature1Sample(int32_t temperature1);
  void addTemperature2Sample(int32_t temperature2);
  void addPressureSample(int32_t pressure);
  // Keeps H1/H2 from the cycle with the worst humidity score.
  void addHumiditySample(int32_t humidity1Value, int32_t humidity2Value, int humidityScore);
  void addCo2Sample(int32_t co2);
  void addGasSample(int32_t gas);
  void addAqiSample(int airQualityScore);
  int32_t meanTemp1() const;
  int32_t meanTemp2() const;
  int32_t meanPressure() const;
  int32_t packedHumidity1() const;
  int32_t packedHumidity2() const;
  int32_t packedGas() const;
  int32_t packedCo2() const;
  int packedAqi() const;
};

#endif
