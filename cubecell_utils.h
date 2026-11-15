#ifndef CUBECELL_UTILS_H
#define CUBECELL_UTILS_H

#include <stdint.h>

// Persist Port-4 sleep intervals across power cycles (SFLASH user rows).
// Returns true if stored intervals were applied to batteryDutyMs / mainsDutyMs.
bool loadPersistedSleepTimes(uint32_t &batteryDutyMs, uint32_t &mainsDutyMs);
void persistSleepTimes(uint16_t batterySeconds, uint16_t mainsSeconds);

#endif
