#ifndef CUBECELL_UTILS_H
#define CUBECELL_UTILS_H

#include <stdint.h>

// Persist Port-4 intervals across power cycles (SFLASH user rows).
// Returns true if stored intervals were applied.
bool loadPersistedSleepTimes(uint32_t &batteryModeMeasureIntervalMs,
                             uint32_t &batteryModeTxIntervalMs,
                             uint32_t &mainsModeMeasureIntervalMs,
                             uint32_t &mainsModeTxIntervalMs);
void persistSleepTimes(uint16_t batteryModeMeasureIntervalSeconds,
                       uint16_t batteryModeTxIntervalSeconds,
                       uint16_t mainsModeMeasureIntervalSeconds,
                       uint16_t mainsModeTxIntervalSeconds);

#endif
