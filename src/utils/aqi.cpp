#include "aqi.h"

#include <Arduino.h>

static int32_t gasBaseline = 0;
static uint8_t gasLearnCount = 0;

void resetGasBaseline() {
  gasBaseline = 0;
  gasLearnCount = 0;
}

bool updateGasBaseline(int32_t gas) {
  if (gas <= 0) {
    return false;
  }
  if (gasLearnCount < GAS_BASELINE_LEARN_SAMPLES) {
    if (gas > gasBaseline) {
      gasBaseline = gas;
    }
    gasLearnCount++;
    Serial.printf("Gas: learning %u/%d, reading=%ld ohm, baseline=%ld ohm (not scored yet)\r\n",
                  gasLearnCount, GAS_BASELINE_LEARN_SAMPLES, (long)gas, (long)gasBaseline);
    return false;
  }
  if (gas > gasBaseline) {
    // Cleaner air: raise baseline by 1/8 of the gap, not in one jump.
    gasBaseline += (gas - gasBaseline) / 8;
  }
  return gasBaseline > 0;
}

int32_t getGasBaseline() {
  return gasBaseline;
}

int calculateGasScore(int32_t gas) {
  if (gasBaseline <= 0 || gas <= 0) {
    return 0;
  }
  if (gas >= gasBaseline) {
    return GAS_SCORE_MAX;
  }
  // Below ~20% of baseline: 0 points.
  if (gas * 5L < gasBaseline) {
    return 0;
  }
  // Integer ratio gas/baseline scaled to 0..GAS_SCORE_MAX, rounded.
  return (int)((gas * (int32_t)GAS_SCORE_MAX + gasBaseline / 2) / gasBaseline);
}

int calculateHumidityScore(float currentHumidity) {
  int humidity = (int)currentHumidity;
  if (humidity < 0) {
    humidity = 0;
  } else if (humidity > 100) {
    humidity = 100;
  }
  if (humidity >= HUM_COMFORT_MIN && humidity <= HUM_COMFORT_MAX) {
    return HUM_SCORE_MAX;
  }
  if (humidity < HUM_COMFORT_MIN) {
    return map(humidity, 0, HUM_COMFORT_MIN, 0, HUM_SCORE_MAX);
  }
  return map(humidity, HUM_COMFORT_MAX, 100, HUM_SCORE_MAX, 0);
}

int scoreHumidity(uint8_t bmeStatus, uint8_t scdStatus, bool restartMeasurement, int32_t humidity1,
                  int32_t humidity2) {
  if (bmeStatus == 1 && scdStatus == 1 && !restartMeasurement) {
    return calculateHumidityScore(round((humidity1 + humidity2) / 2) / 1000.0);
  }
  if (bmeStatus == 1) {
    return calculateHumidityScore(humidity1 / 1000.0);
  }
  if (scdStatus == 1) {
    return calculateHumidityScore(humidity2 / 1000.0);
  }
  return 0;
}

int calculateCo2Score(int32_t co2Ppm) {
  if (co2Ppm <= CO2_GOOD_PPM) {
    return CO2_SCORE_MAX;
  }
  if (co2Ppm >= CO2_POOR_PPM) {
    return 0;
  }
  return map(co2Ppm, CO2_GOOD_PPM, CO2_POOR_PPM, CO2_SCORE_MAX, 0);
}

int combineAirQualityScore(bool hasHumidityMeasurement, int humidityScore, bool hasCo2Measurement, int co2Score,
                           bool hasGasMeasurement, int gasScore) {
  int sum = 0;
  int maxPossible = 0;
  if (hasHumidityMeasurement) {
    sum += humidityScore;
    maxPossible += HUM_SCORE_MAX;
  }
  if (hasCo2Measurement) {
    sum += co2Score;
    maxPossible += CO2_SCORE_MAX;
  }
  if (hasGasMeasurement) {
    sum += gasScore;
    maxPossible += GAS_SCORE_MAX;
  }
  if (maxPossible == 0) {
    return 0;
  }
  if (maxPossible == 100) {
    return sum;
  }
  return sum * 100 / maxPossible;
}

uint32_t colorForAirQuality(int score) {
  if (score <= 33) {
    return COLOR_AQI_RED;
  }
  if (score <= 66) {
    return COLOR_AQI_YELLOW;
  }
  return COLOR_AQI_GREEN;
}
