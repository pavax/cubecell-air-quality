#include "cubecell_utils.h"

#include <Arduino.h>
#include <EEPROM.h>

#define SLEEP_EEPROM_SIZE 10
#define SLEEP_EEPROM_MAGIC 0xA5C6u
#define SLEEP_EEPROM_ADDR 0

struct SleepEeprom {
  uint16_t magic;
  uint16_t batteryModeMeasureIntervalSeconds;
  uint16_t batteryModeTxIntervalSeconds;
  uint16_t mainsModeMeasureIntervalSeconds;
  uint16_t mainsModeTxIntervalSeconds;
};

bool loadPersistedSleepTimes(uint32_t &batteryModeMeasureIntervalMs,
                             uint32_t &batteryModeTxIntervalMs,
                             uint32_t &mainsModeMeasureIntervalMs,
                             uint32_t &mainsModeTxIntervalMs) {
  EEPROM.begin(SLEEP_EEPROM_SIZE);
  SleepEeprom stored = {};
  EEPROM.get(SLEEP_EEPROM_ADDR, stored);
  if (stored.magic != SLEEP_EEPROM_MAGIC ||
      stored.batteryModeMeasureIntervalSeconds == 0 ||
      stored.batteryModeTxIntervalSeconds == 0 ||
      stored.mainsModeMeasureIntervalSeconds == 0 ||
      stored.mainsModeTxIntervalSeconds == 0) {
    return false;
  }

  batteryModeMeasureIntervalMs =
      (uint32_t)stored.batteryModeMeasureIntervalSeconds * 1000UL;
  batteryModeTxIntervalMs = (uint32_t)stored.batteryModeTxIntervalSeconds * 1000UL;
  mainsModeMeasureIntervalMs =
      (uint32_t)stored.mainsModeMeasureIntervalSeconds * 1000UL;
  mainsModeTxIntervalMs = (uint32_t)stored.mainsModeTxIntervalSeconds * 1000UL;
  Serial.printf(
      "Restored intervals bat_measure=%lu ms (%u s) bat_tx=%lu ms (%u s) "
      "mains_measure=%lu ms (%u s) mains_tx=%lu ms (%u s)\r\n",
      (unsigned long)batteryModeMeasureIntervalMs, stored.batteryModeMeasureIntervalSeconds,
      (unsigned long)batteryModeTxIntervalMs, stored.batteryModeTxIntervalSeconds,
      (unsigned long)mainsModeMeasureIntervalMs, stored.mainsModeMeasureIntervalSeconds,
      (unsigned long)mainsModeTxIntervalMs, stored.mainsModeTxIntervalSeconds);
  return true;
}

void persistSleepTimes(uint16_t batteryModeMeasureIntervalSeconds,
                       uint16_t batteryModeTxIntervalSeconds,
                       uint16_t mainsModeMeasureIntervalSeconds,
                       uint16_t mainsModeTxIntervalSeconds) {
  if (batteryModeMeasureIntervalSeconds == 0 || batteryModeTxIntervalSeconds == 0 ||
      mainsModeMeasureIntervalSeconds == 0 || mainsModeTxIntervalSeconds == 0) {
    return;
  }

  EEPROM.begin(SLEEP_EEPROM_SIZE);
  SleepEeprom stored = {
      SLEEP_EEPROM_MAGIC,
      batteryModeMeasureIntervalSeconds,
      batteryModeTxIntervalSeconds,
      mainsModeMeasureIntervalSeconds,
      mainsModeTxIntervalSeconds,
  };
  EEPROM.put(SLEEP_EEPROM_ADDR, stored);
  if (EEPROM.commit()) {
    Serial.printf(
        "Saved intervals bat_measure=%u s bat_tx=%u s mains_measure=%u s mains_tx=%u s\r\n",
        batteryModeMeasureIntervalSeconds, batteryModeTxIntervalSeconds,
        mainsModeMeasureIntervalSeconds, mainsModeTxIntervalSeconds);
  } else {
    Serial.println(F("EEPROM: interval save failed"));
  }
}
