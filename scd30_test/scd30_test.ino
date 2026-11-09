#include "Arduino.h"
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"

SCD30 airSensor;

#define SCD30_MEASUREMENT_INTERVAL 2
#define SCD30_POLL_MS 500
#define SCD30_READ_TIMEOUT_MS 8000
#define POWER_CYCLE_EVERY_N 10

static unsigned int cycle = 0;

void powerSensors(bool on) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, on ? LOW : HIGH);
}

bool probeScd30() {
  Wire.beginTransmission(SCD30_ADDRESS);
  Wire.write(highByte(COMMAND_READ_FW_VER));
  Wire.write(lowByte(COMMAND_READ_FW_VER));
  uint8_t err = Wire.endTransmission();
  Serial.printf("SCD30 probe: err=%u\r\n", err);
  return err == 0;
}

bool startScd30() {
  Serial.println(F("SCD30: connect"));
  if (!airSensor.begin(Wire, false, false)) {
    probeScd30();
    Serial.println(F("SCD30: begin failed"));
    return false;
  }

  airSensor.setAutoSelfCalibration(false);
  airSensor.StopMeasurement();
  delay(500);
  airSensor.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL);
  if (!airSensor.beginMeasuring()) {
    Serial.println(F("SCD30: start failed"));
    return false;
  }
  Serial.println(F("SCD30: measuring"));
  return true;
}

bool readScd30(bool discardFirst) {
  unsigned long start = millis();
  bool discardedFirst = !discardFirst;
  while ((millis() - start) < SCD30_READ_TIMEOUT_MS) {
    if (airSensor.dataAvailable()) {
      int32_t co2 = airSensor.getCO2();
      float temperature = airSensor.getTemperature();
      float humidity = airSensor.getHumidity();
      Serial.printf("SCD30: CO2=%ld ppm T=%.2f C H=%.3f %%\r\n",
                    (long)co2, temperature, humidity);
      if (!discardedFirst) {
        discardedFirst = true;
        Serial.println(F("SCD30: discard first sample"));
        delay(SCD30_POLL_MS);
        continue;
      }
      return true;
    }
    Serial.printf("SCD30: waiting %lu ms\r\n", millis() - start);
    delay(SCD30_POLL_MS);
  }
  Serial.println(F("SCD30: timed out"));
  return false;
}

void busBegin() {
  // Heltec CubeCell: I2C_Stop before I2C_Start after Vext cycles.
  I2C_Stop();
  Wire.begin();
}

void busEnd() {
  Wire.end();
}

void coldStart() {
  Serial.println(F("--- cold start ---"));
  busEnd();
  powerSensors(false);
  delay(500);
  powerSensors(true);
  delay(2000);
  busBegin();
  if (!startScd30()) {
    return;
  }
  readScd30(true);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("SCD30 test"));

  powerSensors(false);
  delay(100);
  coldStart();
}

void loop() {
  cycle++;
  Serial.printf("--- cycle %u ---\r\n", cycle);

  if ((cycle % POWER_CYCLE_EVERY_N) == 0) {
    coldStart();
  } else if (!readScd30(false)) {
    Serial.println(F("SCD30: recover with cold start"));
    coldStart();
  }

  delay(2000);
}
