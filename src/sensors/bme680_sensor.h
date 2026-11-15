#ifndef BME680_SENSOR_H
#define BME680_SENSOR_H

#include <stdint.h>

#define BME_CYCLES_NORMAL 2
#define BME_CYCLES_FOR_GAS_READINGS 5
#define PRESSURE_MIN_PA 70000L
#define PRESSURE_MAX_PA 110000L

bool initBme680(bool heatGas);

bool readBme680(int32_t &temperature, int32_t &humidity, int32_t &pressure, int32_t &gas,
                bool heatGas);

#endif
