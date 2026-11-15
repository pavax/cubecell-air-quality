/*
 * Standalone SCD30 forced-recalibration helper for Heltec CubeCell HTCC-AB01.
 *
 * No LoRaWAN. Powers Vext, prints live CO2, and runs FRC on Serial command.
 *
 * Flash:
 *   .\scripts\cubecell.ps1 -Action flash -Sketch examples/scd30_calibrate
 *
 * Serial 115200. Put the sensor in fresh outdoor air, wait until readings are
 * stable, then send: cal
 *
 * Commands:
 *   help              this text
 *   settings          dump FRC / ASC / interval / firmware
 *   frc               read stored forced-recalibration factor
 *   p <mbar>          set ambient pressure compensation (700-1200), 0 = clear
 *   cal               wait 120 s, FRC to 420 ppm, verify factor and readings
 *   cal <ppm>         same, but FRC to <ppm> (400-2000)
 */

#include "Arduino.h"
#include <Wire.h>
#include "../../src/sensors/scd30.h"

static String lineBuf;
static uint16_t ambientPressureMbar = 0;

void printHelp() {
  Serial.println(F("Commands: help | settings | frc | p <mbar> | cal [ppm]"));
  Serial.println(F("cal: wait 120 s in fresh air, then forced recalibration"));
}

void powerSensorsOn() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  // SCD30 accepts I2C commands about 3 s after power-up.
  delay(3000);
  I2C_Stop();
  Wire.begin();
}

bool printOneSample(const char *tag) {
  unsigned long start = millis();
  while ((millis() - start) < 5000UL) {
    uint16_t co2 = 0;
    float t = 0;
    float h = 0;
    if (pollScd30(co2, t, h)) {
      if (tag != nullptr && tag[0] != '\0') {
        Serial.printf("%s CO2=%u ppm T=%.2f C H=%.2f %%\r\n", tag, co2, t, h);
      } else {
        Serial.printf("CO2=%u ppm T=%.2f C H=%.2f %%\r\n", co2, t, h);
      }
      return true;
    }
    delay(100);
  }
  Serial.println(F("SCD30: no sample within 5 s"));
  return false;
}

void runCalibration(uint16_t refPpm) {
  if (refPpm < 400 || refPpm > 2000) {
    Serial.println(F("FRC reference must be 400-2000 ppm"));
    return;
  }

  Serial.println(F("Sample before wait:"));
  printOneSample("before-wait");

  if (!calibrateScd30(refPpm)) {
    Serial.println(F("Calibration failed."));
    return;
  }

  Serial.println(F("Samples after FRC (should be near reference in fresh air):"));
  for (int i = 0; i < 5; i++) {
    printOneSample("after-frc");
  }
  Serial.println(F("Done. Live readings continue; send cal again if needed."));
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String lower = line;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") {
    printHelp();
    return;
  }
  if (lower == "settings") {
    printScd30Settings(ambientPressureMbar);
    return;
  }
  if (lower == "frc") {
    uint16_t val = 0;
    if (getScd30ForcedRecalibration(val)) {
      Serial.printf("FRC factor=%u ppm\r\n", val);
    } else {
      Serial.println(F("getForcedRecalibration failed"));
    }
    return;
  }
  if (lower == "cal") {
    runCalibration(SCD30_FRC_PPM);
    return;
  }
  if (lower.startsWith("cal ")) {
    uint16_t ppm = (uint16_t)lower.substring(4).toInt();
    runCalibration(ppm);
    return;
  }
  if (lower.startsWith("p ")) {
    uint16_t mbar = (uint16_t)lower.substring(2).toInt();
    if (setScd30AmbientPressure(mbar)) {
      ambientPressureMbar = mbar;
      Serial.printf("Ambient pressure set to %u mbar\r\n", ambientPressureMbar);
    } else {
      Serial.println(F("setAmbientPressure failed"));
    }
    return;
  }

  Serial.println(F("Unknown command. Send help"));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("SCD30 calibrate sketch"));

  powerSensorsOn();
  if (!initScd30(0)) {
    Serial.println(F("SCD30 not detected. Check wiring / Vext. Halted."));
    while (true) {
      delay(1000);
    }
  }

  printScd30Settings(ambientPressureMbar);
  printHelp();
  Serial.println(F("Live CO2 below. Fresh air, then: cal"));
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuf.length() > 0) {
        handleLine(lineBuf);
        lineBuf = "";
      }
    } else if (lineBuf.length() < 48) {
      lineBuf += c;
    }
  }

  uint16_t co2 = 0;
  float t = 0;
  float h = 0;
  if (pollScd30(co2, t, h)) {
    Serial.printf("CO2=%u ppm T=%.2f C H=%.2f %%\r\n", co2, t, h);
  }
}
