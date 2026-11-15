#ifndef SCD30_SENSOR_H
#define SCD30_SENSOR_H

#include <stdint.h>

#define SCD30_MEASUREMENT_INTERVAL 2
#define SCD30_READ_TIMEOUT_MS 8000
#define SCD30_POLL_MS 500
#define SCD30_CALIBRATION_TIME_S 120
// Outdoor fresh-air reference for forced recalibration (current ambient ~420 ppm).
#define SCD30_FRC_PPM 420
#define SCD30_PRESSURE_MIN_MBAR 700
#define SCD30_PRESSURE_MAX_MBAR 1200

// Pascals to mbar. Outside 700–1200 mbar the value is clamped.
uint16_t toPressureMbar(int32_t pressurePa);

// Connect only. SparkFun begin(measBegin=true) starts a measurement and then
// changes the interval, which the SCD30 ignores unless measurement is stopped.
bool attachScd30();

bool initScd30(uint16_t pressureMbar);

// Ambient pressure compensation. 0 clears it (altitude / no continuous pressure).
bool setScd30AmbientPressure(uint16_t pressureMbar);

bool getScd30ForcedRecalibration(uint16_t &ppm);

void printScd30Settings(uint16_t ambientPressureMbar);

// True when a fresh sample is ready; fills co2 / temperature C / humidity %.
bool pollScd30(uint16_t &co2, float &temperature, float &humidity);

// SCD30 must already be measuring. Waits in fresh air, then applies FRC. No RGB
// during the wait: NeoPixel bitbang shares the I2C port and corrupts FRC.
bool calibrateScd30(uint16_t refPpm = SCD30_FRC_PPM);

// Returns 1 on success, 9 on timeout. Caller uses 0 when the sensor is missing.
uint8_t readScd30(int32_t &co2, int32_t &temperature, int32_t &humidity, bool discardFirst);

#endif
