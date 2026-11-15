#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "BME680.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "credentials.h"
#include "src/utils/cubecell_utils.h"
#include "src/utils/window_accum.h"

BME680_Class BME680;
SCD30 airSensor;

#define SCD30_MEASUREMENT_INTERVAL 2
#define SCD30_READ_TIMEOUT_MS 8000
#define SCD30_POLL_MS 500
#define SCD30_CALIBRATION_TIME_S 120
#define BME_CYCLES_NORMAL 2
#define BME_CYCLES_FOR_GAS_READINGS 5
#define WAKE_UP_PIN USER_KEY
#define DEFAULT_BATTERY_MODE_MEASURE_INTERVAL_MS (1000UL * 60 * 60)
#define DEFAULT_BATTERY_MODE_TX_INTERVAL_MS (1000UL * 60 * 60 * 4)
#define DEFAULT_MAINS_MODE_MEASURE_INTERVAL_MS (1000UL * 60 * 5)
#define DEFAULT_MAINS_MODE_TX_INTERVAL_MS (1000UL * 60 * 60)
#define PRESSURE_MIN_PA 70000L
#define PRESSURE_MAX_PA 110000L
#define PRESSURE_CHANGE_THRESHOLD_MBAR 10
#define SCD30_PRESSURE_MIN_MBAR 700
#define SCD30_PRESSURE_MAX_MBAR 1200
#define SENSOR_POWER_OFF_MS 4000
#define SENSOR_ATTEMPTS 3
// Extra time beyond the measure interval before a hung cycle forces HW_Reset.
#define CYCLE_WATCHDOG_MARGIN_MS (1000UL * 60 * 60)

#define HUM_COMFORT_MIN 30
#define HUM_COMFORT_MAX 60
#define HUM_SCORE_MAX 25
#define CO2_GOOD_PPM 800
#define CO2_POOR_PPM 2000
#define CO2_SCORE_MAX 50
#define GAS_SCORE_MAX 25
#define GAS_BASELINE_LEARN_SAMPLES 3

// 0xRRGGBB. Cyan while measuring, independent of the core's colors.
#define COLOR_MEASURE 0x005050

// AQI sleep colors, same brightness as COLOR_MEASURE.
#define COLOR_AQI_RED 0x500000
#define COLOR_AQI_YELLOW 0x505000
#define COLOR_AQI_GREEN 0x005000

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/* Sleep/measure interval and separate uplink interval per mode, in [ms]. */
uint32_t batteryModeMeasureIntervalMs = DEFAULT_BATTERY_MODE_MEASURE_INTERVAL_MS;
uint32_t batteryModeTxIntervalMs = DEFAULT_BATTERY_MODE_TX_INTERVAL_MS;
uint32_t mainsModeMeasureIntervalMs = DEFAULT_MAINS_MODE_MEASURE_INTERVAL_MS;
uint32_t mainsModeTxIntervalMs = DEFAULT_MAINS_MODE_TX_INTERVAL_MS;
uint32_t appTxDutyCycle = DEFAULT_BATTERY_MODE_MEASURE_INTERVAL_MS;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

/* Number of confirmed uplink retries. The MAC lowers the datarate after repeated misses. */
uint8_t confirmedNbTrials = 8;

static unsigned int batteryVoltage;
static unsigned int uptimeCount;
static bool mainsConnected = false;
static volatile bool userKeyWakePending = false;
static bool forceUplink = false;
// 0xFFFFFFFF: first cycle after boot/join always uplinks.
static const uint32_t MS_SINCE_UPLINK_UNSET = 0xFFFFFFFFu;
static uint32_t msSinceLastUplink = MS_SINCE_UPLINK_UNSET;
static bool recalibrateDuty = false;
static uint16_t appliedPressureMbar = 0;
static bool keepSensorsPoweredOn = false;
static int32_t gasBaseline = 0;
static uint8_t gasLearnCount = 0;
// -1: no valid AQI component. Otherwise AQI from 0 to 100.
static int sleepAirQualityScore = -1;

static WindowAccum windowAccum;

static TimerEvent_t cycleWatchdogTimer;

uint32_t activeMeasureIntervalMs() {
  return mainsConnected ? mainsModeMeasureIntervalMs : batteryModeMeasureIntervalMs;
}

uint32_t activeTxIntervalMs() {
  return mainsConnected ? mainsModeTxIntervalMs : batteryModeTxIntervalMs;
}

void onCycleTimeoutWatchdog() {
  Serial.println(F("Watchdog: Cycle Timed Out"));
  delay(100);
  HW_Reset(0);
}

void resetCycleWatchdog(uint32_t measureIntervalMs) {
  uint32_t watchdogMs = measureIntervalMs + CYCLE_WATCHDOG_MARGIN_MS;
  Serial.printf("Watchdog: reset %lu s\r\n", (unsigned long)(watchdogMs / 1000));
  TimerStop(&cycleWatchdogTimer);
  TimerSetValue(&cycleWatchdogTimer, watchdogMs);
  TimerStart(&cycleWatchdogTimer);
}

void resetGasBaseline() {
  gasBaseline = 0;
  gasLearnCount = 0;
}

// Learns or raises the baseline. true = scoring allowed (after learn phase).
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

int scoreHumidity(uint8_t bmeStatus, uint8_t scdStatus, bool restartMeasurement, int32_t humidity1, int32_t humidity2) {
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

// 0–33 red, 34–66 yellow, 67–100 green.
uint32_t colorForAirQuality(int score) {
  if (score <= 33) {
    return COLOR_AQI_RED;
  }
  if (score <= 66) {
    return COLOR_AQI_YELLOW;
  }
  return COLOR_AQI_GREEN;
}

void putUint16(uint8_t offset, uint16_t value) {
  appData[offset] = highByte(value);
  appData[offset + 1] = lowByte(value);
}

void packWindowIntoAppData(const WindowAccum &window, unsigned int uplinkCount,
                           uint8_t sensorStatus, bool mains, int32_t gasBaselineOhm) {
  int32_t packTemp1 = window.meanTemp1();
  int32_t packTemp2 = window.meanTemp2();
  int32_t packPressure = window.meanPressure();
  int32_t packHumidity1 = window.packedHumidity1();
  int32_t packHumidity2 = window.packedHumidity2();
  int32_t packGas = window.packedGas();
  int32_t packCo2 = window.packedCo2();
  int packAqi = window.packedAqi();

  appDataSize = 26;
  putUint16(0, uplinkCount);
  putUint16(2, window.batteryMv);
  putUint16(4, packTemp1);
  putUint16(6, packTemp2);
  putUint16(8, round(packHumidity1 / 10));
  putUint16(10, round(packHumidity2 / 10));
  appData[12] = (uint32_t)packPressure >> 24;
  appData[13] = (uint32_t)packPressure >> 16;
  appData[14] = (uint32_t)packPressure >> 8;
  appData[15] = (uint32_t)packPressure & 0xFF;
  putUint16(16, round(packGas / 10));
  putUint16(18, packAqi);
  putUint16(20, packCo2);
  appData[22] = sensorStatus;
  appData[23] = mains ? 1 : 0;
  putUint16(24, round(gasBaselineOhm / 10));

  Serial.printf(
      "Window T1=%ld T2=%ld H1=%ld H2=%ld P=%ld Gas=%ld Base=%ld CO2=%ld AQI=%d Bat=%u St=%u Mains=%u\r\n",
      (long)packTemp1, (long)packTemp2, (long)packHumidity1, (long)packHumidity2, (long)packPressure,
      (long)packGas, (long)gasBaselineOhm, (long)packCo2, packAqi, window.batteryMv, sensorStatus,
      mains ? 1 : 0);
}

bool powerSensorsOn() {
  // The LoRaWAN stack may drive Vext during join, so the pin level is not our power state.
  bool needsColdStart = !keepSensorsPoweredOn;
  digitalWrite(Vext, LOW);
  if (needsColdStart) {
    // SCD30 accepts I2C commands about 3 s after power-up.
    delay(3000);
  }
  // CubeCell: after Vext cycles, I2C_Stop before Wire.begin()/I2C_Start is needed.
  I2C_Stop();
  Wire.begin();
  return needsColdStart;
}

// Bus off and power cut. The next power-on is a cold start.
void cutSensorPower() {
  Wire.end();
  digitalWrite(Vext, HIGH);
  LoRaWAN.holdVext(false);
  keepSensorsPoweredOn = false;
  resetGasBaseline();
}

// Bus off. keepPowered leaves the rail on for the next warm cycle.
void parkSensors(bool keepPowered) {
  Wire.end();
  if (keepPowered) {
    digitalWrite(Vext, LOW);
    LoRaWAN.holdVext(true);
    keepSensorsPoweredOn = true;
    return;
  }
  digitalWrite(Vext, HIGH);
  LoRaWAN.holdVext(false);
  keepSensorsPoweredOn = false;
  appliedPressureMbar = 0;
}

// Pascals to mbar. Outside 700–1200 mbar the value is clamped.
uint16_t toPressureMbar(int32_t pressurePa) {
  int32_t pressureMbar = pressurePa / 100;
  if (pressureMbar < SCD30_PRESSURE_MIN_MBAR) {
    return SCD30_PRESSURE_MIN_MBAR;
  }
  if (pressureMbar > SCD30_PRESSURE_MAX_MBAR) {
    return SCD30_PRESSURE_MAX_MBAR;
  }
  return pressureMbar;
}

bool hasPressureSignificantlyChanged(uint16_t pressureMbar) {
  if (pressureMbar == 0 || appliedPressureMbar == 0) {
    return pressureMbar != 0;
  }
  return abs((int)pressureMbar - (int)appliedPressureMbar) >= PRESSURE_CHANGE_THRESHOLD_MBAR;
}

bool initBme680(bool heatGas) {
  Serial.println(F("BME680: init"));
  if (!BME680.begin(I2C_STANDARD_MODE)) {
    Serial.println(F("BME680: not available"));
    return false;
  }
  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
  BME680.setIIRFilter(IIR4);
  if (heatGas) {
    BME680.setGas(320, 150);
  } else {
    BME680.setGas(0, 0);
  }
  Serial.printf("BME680: ready, heater=%d\r\n", heatGas ? 1 : 0);
  return true;
}

bool readBme680(int32_t &temperature, int32_t &humidity, int32_t &pressure, int32_t &gas, bool heatGas) {
  bool haveReading = false; 
  int cycles = heatGas ? BME_CYCLES_FOR_GAS_READINGS : BME_CYCLES_NORMAL;
  gas = 0;
  for (int i = 0; i < cycles; i++) {
    int32_t sampleTemperature = 0;
    int32_t sampleHumidity = 0;
    int32_t samplePressure = 0;
    int32_t sampleGas = 0;
    Serial.printf("BME680: wait %d/%d\r\n", i + 1, cycles);
    BME680.getSensorData(sampleTemperature, sampleHumidity, samplePressure, sampleGas, true);
    Serial.printf("BME680 %d/%d: T=%ld.%02ld C H=%ld.%03ld %% P=%ld.%02ld hPa Gas=%ld ohm\r\n",
                  i + 1, cycles,
                  (long)(sampleTemperature / 100), (long)abs(sampleTemperature % 100),
                  (long)(sampleHumidity / 1000), (long)abs(sampleHumidity % 1000),
                  (long)(samplePressure / 100), (long)(samplePressure % 100), (long)sampleGas);
    // The first sample is not used. The last plausible one is kept.
    if (i == 0) {
      continue;
    }
    bool plausible = sampleTemperature > -2000 && sampleTemperature < 6000 && sampleHumidity > 0 &&
                     sampleHumidity < 100000 && samplePressure >= PRESSURE_MIN_PA && samplePressure <= PRESSURE_MAX_PA &&
                     (!heatGas || sampleGas > 0);
    if (plausible) {
      temperature = sampleTemperature;
      humidity = sampleHumidity;
      pressure = samplePressure;
      gas = heatGas ? sampleGas : 0;
      haveReading = true;
    }
  }
  if (!haveReading) {
    Serial.println(F("BME680: no valid reading"));
  }
  return haveReading;
}

// Connect only. SparkFun begin(measBegin=true) starts a measurement and then
// changes the interval, which the SCD30 ignores unless measurement is stopped.
bool attachScd30() {
  if (!airSensor.begin(Wire, false, false)) {
    return false;
  }
  airSensor.setAutoSelfCalibration(false);
  return true;
}

bool initScd30(uint16_t pressureMbar) {
  Serial.printf("SCD30: connect, pressure=%u mbar\r\n", pressureMbar);
  if (!attachScd30()) {
    Wire.beginTransmission(SCD30_ADDRESS);
    Wire.write(highByte(COMMAND_READ_FW_VER));
    Wire.write(lowByte(COMMAND_READ_FW_VER));
    uint8_t err = Wire.endTransmission();
    Serial.printf("SCD30: not available, fw err=%u\r\n", err);
    return false;
  }

  airSensor.StopMeasurement();
  delay(500);
  Serial.printf("SCD30: interval %d s\r\n", SCD30_MEASUREMENT_INTERVAL);
  airSensor.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL);
  bool started = pressureMbar == 0 ? airSensor.beginMeasuring() : airSensor.beginMeasuring(pressureMbar);
  if (!started) {
    Serial.println(F("SCD30: start failed"));
    return false;
  }
  Serial.println(F("SCD30: measuring"));
  return true;
}

void calibrateScd30() {
  Serial.println(F("SCD30: calibrating for 120s in fresh air"));
  delay(SCD30_CALIBRATION_TIME_S * 1000);
  airSensor.setForcedRecalibrationFactor(400);
  airSensor.setAutoSelfCalibration(false);
  recalibrateDuty = false;
  Serial.println(F("SCD30: calibration set to 400 ppm"));
}

// Returns 1 on success, 9 on timeout. Caller uses 0 when the sensor is missing.
uint8_t readScd30(int32_t &co2, int32_t &temperature, int32_t &humidity, bool discardFirst) {
  unsigned long start = millis();
  bool discardedFirst = !discardFirst;
  while ((millis() - start) < SCD30_READ_TIMEOUT_MS) {
    if (airSensor.dataAvailable()) {
      int32_t sampleCo2 = airSensor.getCO2();
      int32_t sampleTemperature = airSensor.getTemperature() * 100;
      int32_t sampleHumidity = airSensor.getHumidity() * 1000;
      Serial.printf("SCD30: CO2=%ld ppm T=%ld.%02ld C H=%ld.%03ld %%\r\n",
                    (long)sampleCo2, (long)(sampleTemperature / 100), (long)abs(sampleTemperature % 100),
                    (long)(sampleHumidity / 1000), (long)abs(sampleHumidity % 1000));
      // The first sample after a fresh start has not settled.
      if (!discardedFirst) {
        discardedFirst = true;
        Serial.println(F("SCD30: discard first sample"));
        delay(SCD30_POLL_MS);
        continue;
      }
      co2 = sampleCo2;
      temperature = sampleTemperature;
      humidity = sampleHumidity;
      return 1;
    }
    Serial.printf("SCD30: waiting %lu ms\r\n", millis() - start);
    delay(SCD30_POLL_MS);
  }
  Serial.println(F("SCD30: timed out"));
  return 9;
}

void prepareTxFrame(uint8_t port) {
  (void)port;

  detachInterrupt(WAKE_UP_PIN);
  batteryVoltage = getBatteryVoltage();
  mainsConnected = digitalRead(GPIO3) == HIGH;
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

  appTxDutyCycle = activeMeasureIntervalMs();
  // After mode detect: hang during measure trips reset before the next wake is due.
  resetCycleWatchdog(appTxDutyCycle);

  // Debug: LED stays on while awake, including on battery.
  LoRaWAN.enableRgb();

  int32_t temperature1 = 0;
  int32_t temperature2 = 0;
  int32_t humidity1 = 0;
  int32_t humidity2 = 0;
  int32_t pressure = 0;
  int32_t gas = 0;
  int32_t co2 = 0;
  uint8_t bmeStatus = 0;
  uint8_t scdStatus = 0;

  if (!mainsConnected) {
    resetGasBaseline();
  }

  Serial.printf(
      "Measure uptime=%u battery=%u mV mains=%u measure=%lu s tx=%lu s since_tx=%lu s\r\n",
      uptimeCount, batteryVoltage, mainsConnected ? 1 : 0,
      (unsigned long)(appTxDutyCycle / 1000),
      (unsigned long)(activeTxIntervalMs() / 1000),
      (unsigned long)(msSinceLastUplink == MS_SINCE_UPLINK_UNSET ? 0 : msSinceLastUplink / 1000));

  // RGB and I2C share GPIO port 0. Lighting the LED writes the whole port, so do it before Wire.begin().
  if (LoRaWAN.isRgbEnabled()) {
    turnOnRGB(COLOR_MEASURE, 0);
  }
  bool needsColdStart = powerSensorsOn();
  bool restartMeasurement;
  for (int attempt = 0; attempt < SENSOR_ATTEMPTS; attempt++) {
    if (attempt > 0) {
      unsigned long powerOffMs = (unsigned long)SENSOR_POWER_OFF_MS * attempt;
      Serial.printf("Sensors: fail, power cycle %lu ms\r\n", powerOffMs);
      cutSensorPower();
      delay(powerOffMs);
      appliedPressureMbar = 0;
      needsColdStart = powerSensorsOn();
    }

    bmeStatus = 0;
    scdStatus = 0;
    if (initBme680(mainsConnected) && readBme680(temperature1, humidity1, pressure, gas, mainsConnected)) {
      bmeStatus = 1;
    }

    uint16_t pressureMbar = bmeStatus == 1 ? toPressureMbar(pressure) : 0;
    bool pressureChanged = hasPressureSignificantlyChanged(pressureMbar);

    restartMeasurement = needsColdStart || pressureChanged;
    bool scdReady;
    if (restartMeasurement) {
      scdReady = initScd30(pressureMbar);
      if (scdReady) {
        appliedPressureMbar = pressureMbar;
      }
    } else {
      scdReady = attachScd30();
    }
    if (scdReady) {
      if (recalibrateDuty) {
        calibrateScd30();
      }
      scdStatus = readScd30(co2, temperature2, humidity2, restartMeasurement);
    }

    if (bmeStatus == 1 && scdStatus == 1) {
      break;
    }
  }

  // Mains keeps the rail on only when at least one sensor answered; otherwise force a cold start next cycle.
  bool keepPowered = mainsConnected && (bmeStatus == 1 || scdStatus == 1);
  parkSensors(keepPowered);

  bool hasHumidityMeasurement = bmeStatus == 1 || scdStatus == 1;
  int humidityScore = scoreHumidity(bmeStatus, scdStatus, restartMeasurement, humidity1, humidity2);
  bool hasCo2Measurement = scdStatus == 1;
  int co2Score = hasCo2Measurement ? calculateCo2Score(co2) : 0;
  bool hasGasMeasurement = mainsConnected && gas > 0 && updateGasBaseline(gas);
  int gasScore = hasGasMeasurement ? calculateGasScore(gas) : 0;
  int airQualityScore = combineAirQualityScore(hasHumidityMeasurement, humidityScore, hasCo2Measurement, co2Score,
                                              hasGasMeasurement, gasScore);
  sleepAirQualityScore = (hasHumidityMeasurement || hasCo2Measurement || hasGasMeasurement) ? airQualityScore : -1;

  Serial.printf("AQI humidity=%d/%d co2=%d/%d gas=%d/%d total=%d baseline=%ld status=%u\r\n",
                humidityScore, HUM_SCORE_MAX, co2Score, CO2_SCORE_MAX, gasScore, GAS_SCORE_MAX,
                airQualityScore, (long)gasBaseline, bmeStatus * 10 + scdStatus);

  windowAccum.addBatterySample(batteryVoltage);
  if (bmeStatus == 1) {
    windowAccum.addTemperature1Sample(temperature1);
    windowAccum.addPressureSample(pressure);
  }
  if (scdStatus == 1) {
    windowAccum.addTemperature2Sample(temperature2);
  }
  if (hasHumidityMeasurement) {
    windowAccum.addHumiditySample(humidity1, humidity2, humidityScore);
  }
  if (hasCo2Measurement) {
    windowAccum.addCo2Sample(co2);
  }
  if (gas > 0) {
    windowAccum.addGasSample(gas);
  }
  if (hasHumidityMeasurement || hasCo2Measurement || hasGasMeasurement) {
    windowAccum.addAqiSample(airQualityScore);
  }

  packWindowIntoAppData(windowAccum, uptimeCount, bmeStatus * 10 + scdStatus, mainsConnected, gasBaseline);
}

bool uplinkDue() {
  if (forceUplink) {
    return true;
  }
  return msSinceLastUplink >= activeTxIntervalMs();
}

void showSleepAirQuality() {
  if (!mainsConnected || sleepAirQualityScore < 0) {
    return;
  }
  // Must run on every deep-sleep entry: MAC turnOffRGB after TX/RX can clear
  // the LED after an earlier pass. Core turnOnRGB skips delay when vextHoldLow.
  turnOnRGB(colorForAirQuality(sleepAirQualityScore), 0);
}

// After MAC SEND/RX/ACK LED use, immediately before CySysPmDeepSleep.
extern "C" void onEnterLowPower(void) {
  showSleepAirQuality();
}

void prepareBeforeSleep() {
  if (keepSensorsPoweredOn) {
    LoRaWAN.holdVext(true);
    Serial.println(F("Sleep: Vext stays on"));
  } else {
    LoRaWAN.holdVext(false);
    digitalWrite(Vext, HIGH);
    Serial.println(F("Sleep: Vext turned off"));
  }
  delay(50);
}

void onWakeUp() {
  if (deviceState == DEVICE_STATE_SLEEP && digitalRead(WAKE_UP_PIN) == HIGH) {
    userKeyWakePending = true;
  }
}

void showBootColors() {
  // Startup only. 4 × 400 ms, then off. Join sets its own colors.
  turnOnRGB(COLOR_SEND, 400);
  turnOnRGB(COLOR_RXWINDOW2, 400);
  turnOnRGB(COLOR_RECEIVED, 400);
  turnOnRGB(COLOR_RXWINDOW1, 400);
  turnOffRGB();
}

void setup() {
  Serial.begin(115200);
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
  pinMode(GPIO3, INPUT);

#if (AT_SUPPORT)
  enableAt();
#endif

  LoRaWAN.enableRgb();
  showBootColors();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();

  userKeyWakePending = false;
  windowAccum.reset();
  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

  TimerInit(&cycleWatchdogTimer, onCycleTimeoutWatchdog);
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT: {
#if (LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
#if (AT_SUPPORT)
      getDevParam();
#endif
      // After getDevParam so Port-4 intervals beat AT/default DutyCycle.
      loadPersistedSleepTimes(batteryModeMeasureIntervalMs, batteryModeTxIntervalMs,
                              mainsModeMeasureIntervalMs, mainsModeTxIntervalMs);
      mainsConnected = digitalRead(GPIO3) == HIGH;
      appTxDutyCycle = activeMeasureIntervalMs();
      printDevParam();
      LoRaWAN.init(loraWanClass, loraWanRegion);
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN: {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND: {
      prepareTxFrame(appPort);
      if (uplinkDue()) {
        LoRaWAN.send();
        uptimeCount++;
        msSinceLastUplink = 0;
        forceUplink = false;
        windowAccum.reset();
      } else {
        Serial.println(F("Uplink skipped (measure only)"));
      }
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE: {
      txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
      if (msSinceLastUplink != MS_SINCE_UPLINK_UNSET) {
        uint32_t nextSince = msSinceLastUplink + txDutyCycleTime;
        msSinceLastUplink =
            nextSince < msSinceLastUplink ? MS_SINCE_UPLINK_UNSET - 1 : nextSince;
      }
      LoRaWAN.cycle(txDutyCycleTime);
      Serial.printf("Sleep %d s\r\n", (int)(txDutyCycleTime / 1000));
      prepareBeforeSleep();
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP: {
      if (userKeyWakePending) {
        Serial.println(F("Wakeup: send now"));
        forceUplink = true;
        LoRaWAN.txNextPacket();
        userKeyWakePending = false;
      } else {
        LoRaWAN.sleep();
      }
      break;
    }
    default: {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}

// AT+Reclibrate=1 calibrates the SCD30 on the next measurement.
bool checkUserAt(char *cmd, char *content) {
  (void)content;
  if (strcmp(cmd, "Reclibrate") == 0) {
    recalibrateDuty = true;
    Serial.println(F("SCD30: calibrate next cycle"));
    return true;
  }
  return false;
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  Serial.printf("+REV DATA: %s, RXSIZE: %d, PORT: %d\r\n",
                mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1",
                mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print(F("+REV DATA: "));
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();

  if (mcpsIndication->Port == 4 && mcpsIndication->BufferSize >= 8) {
    uint16_t batteryModeMeasureIntervalSeconds =
        (uint16_t)(mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8));
    uint16_t batteryModeTxIntervalSeconds =
        (uint16_t)(mcpsIndication->Buffer[3] | (mcpsIndication->Buffer[2] << 8));
    uint16_t mainsModeMeasureIntervalSeconds =
        (uint16_t)(mcpsIndication->Buffer[5] | (mcpsIndication->Buffer[4] << 8));
    uint16_t mainsModeTxIntervalSeconds =
        (uint16_t)(mcpsIndication->Buffer[7] | (mcpsIndication->Buffer[6] << 8));
    if (batteryModeMeasureIntervalSeconds == 0 || batteryModeTxIntervalSeconds == 0 ||
        mainsModeMeasureIntervalSeconds == 0 || mainsModeTxIntervalSeconds == 0) {
      Serial.println(F("Intervals ignored (0 s)"));
      return;
    }
    if (batteryModeTxIntervalSeconds < batteryModeMeasureIntervalSeconds) {
      Serial.printf("battery_tx raised from %u s to measure %u s\r\n",
                    batteryModeTxIntervalSeconds, batteryModeMeasureIntervalSeconds);
      batteryModeTxIntervalSeconds = batteryModeMeasureIntervalSeconds;
    }
    if (mainsModeTxIntervalSeconds < mainsModeMeasureIntervalSeconds) {
      Serial.printf("mains_tx raised from %u s to measure %u s\r\n",
                    mainsModeTxIntervalSeconds, mainsModeMeasureIntervalSeconds);
      mainsModeTxIntervalSeconds = mainsModeMeasureIntervalSeconds;
    }
    batteryModeMeasureIntervalMs = (uint32_t)batteryModeMeasureIntervalSeconds * 1000UL;
    batteryModeTxIntervalMs = (uint32_t)batteryModeTxIntervalSeconds * 1000UL;
    mainsModeMeasureIntervalMs = (uint32_t)mainsModeMeasureIntervalSeconds * 1000UL;
    mainsModeTxIntervalMs = (uint32_t)mainsModeTxIntervalSeconds * 1000UL;
    appTxDutyCycle = activeMeasureIntervalMs();
    persistSleepTimes(batteryModeMeasureIntervalSeconds, batteryModeTxIntervalSeconds,
                      mainsModeMeasureIntervalSeconds, mainsModeTxIntervalSeconds);
    Serial.printf(
        "Intervals bat_measure=%lu ms bat_tx=%lu ms mains_measure=%lu ms mains_tx=%lu ms (active=%lu ms)\r\n",
        (unsigned long)batteryModeMeasureIntervalMs, (unsigned long)batteryModeTxIntervalMs,
        (unsigned long)mainsModeMeasureIntervalMs, (unsigned long)mainsModeTxIntervalMs,
        (unsigned long)appTxDutyCycle);
  } else if (mcpsIndication->Port == 5) {
    recalibrateDuty = true;
    Serial.println(F("SCD30: calibrate next cycle"));
  } else if (mcpsIndication->Port == 9) {
    Serial.println(F("Reset"));
    HW_Reset(0);
  }
}
