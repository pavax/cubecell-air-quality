#ifndef AQI_H
#define AQI_H

#include <stdint.h>

#define HUM_COMFORT_MIN 30
#define HUM_COMFORT_MAX 60
#define HUM_SCORE_MAX 25
#define CO2_GOOD_PPM 800
#define CO2_POOR_PPM 2000
#define CO2_SCORE_MAX 50
#define GAS_SCORE_MAX 25
#define GAS_BASELINE_LEARN_SAMPLES 3

// AQI sleep colors (0xRRGGBB).
#define COLOR_AQI_RED 0x500000
#define COLOR_AQI_YELLOW 0x505000
#define COLOR_AQI_GREEN 0x005000

void resetGasBaseline();
// Learns or raises the baseline. true = scoring allowed (after learn phase).
bool updateGasBaseline(int32_t gas);
int32_t getGasBaseline();

int calculateGasScore(int32_t gas);
int calculateHumidityScore(float currentHumidity);
int scoreHumidity(uint8_t bmeStatus, uint8_t scdStatus, bool restartMeasurement, int32_t humidity1,
                  int32_t humidity2);
int calculateCo2Score(int32_t co2Ppm);
int combineAirQualityScore(bool hasHumidityMeasurement, int humidityScore, bool hasCo2Measurement, int co2Score,
                           bool hasGasMeasurement, int gasScore);

// 0–33 red, 34–66 yellow, 67–100 green.
uint32_t colorForAirQuality(int score);

#endif
