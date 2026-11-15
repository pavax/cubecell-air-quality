#include "cubecell_utils.h"

#include <Arduino.h>
#include <EEPROM.h>

#define SLEEP_EEPROM_SIZE 8
#define SLEEP_EEPROM_MAGIC 0xA5C4u
#define SLEEP_EEPROM_ADDR 0

struct SleepEeprom {
  uint16_t magic;
  uint16_t batterySeconds;
  uint16_t mainsSeconds;
};

bool loadPersistedSleepTimes(uint32_t &batteryDutyMs, uint32_t &mainsDutyMs) {
  EEPROM.begin(SLEEP_EEPROM_SIZE);
  SleepEeprom stored = {};
  EEPROM.get(SLEEP_EEPROM_ADDR, stored);
  if (stored.magic != SLEEP_EEPROM_MAGIC || stored.batterySeconds == 0 ||
      stored.mainsSeconds == 0) {
    return false;
  }

  batteryDutyMs = (uint32_t)stored.batterySeconds * 1000UL;
  mainsDutyMs = (uint32_t)stored.mainsSeconds * 1000UL;
  Serial.printf("Restored duty cycles battery=%lu ms (%u s) mains=%lu ms (%u s)\r\n",
                (unsigned long)batteryDutyMs, stored.batterySeconds,
                (unsigned long)mainsDutyMs, stored.mainsSeconds);
  return true;
}

void persistSleepTimes(uint16_t batterySeconds, uint16_t mainsSeconds) {
  if (batterySeconds == 0 || mainsSeconds == 0) {
    return;
  }

  EEPROM.begin(SLEEP_EEPROM_SIZE);
  SleepEeprom stored = { SLEEP_EEPROM_MAGIC, batterySeconds, mainsSeconds };
  EEPROM.put(SLEEP_EEPROM_ADDR, stored);
  if (EEPROM.commit()) {
    Serial.printf("Saved duty cycles battery=%u s mains=%u s\r\n",
                  batterySeconds, mainsSeconds);
  } else {
    Serial.println(F("EEPROM: duty cycle save failed"));
  }
}
