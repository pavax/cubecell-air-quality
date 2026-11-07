#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "BME680.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "credentials.h"

BME680_Class BME680;
SCD30 airSensor;

#define SCD30_MEASUREMENT_INTERVAL 2
#define SCD30_READ_TIMEOUT_MS 8000
#define SCD30_POLL_MS 500
#define SCD30_CALIBRATION_TIME_S 120
#define BME_CYCLES_NORMAL 2
#define BME_CYCLES_FOR_GAS_READINGS 5
#define WAKE_UP_PIN USER_KEY
#define BATTERY_TX_DUTY_MS (1000UL * 60 * 30)
#define MAINS_TX_DUTY_MS (1000UL * 60 * 5)
#define PRESSURE_MIN_PA 70000L
#define PRESSURE_MAX_PA 110000L
#define PRESSURE_CHANGE_THRESHOLD_MBAR 10
#define SCD30_PRESSURE_MIN_MBAR 700
#define SCD30_PRESSURE_MAX_MBAR 1200

#define HUM_COMFORT_MIN 30
#define HUM_COMFORT_MAX 60
#define HUM_SCORE_MAX 30
#define CO2_GOOD_PPM 800
#define CO2_POOR_PPM 2000
#define CO2_SCORE_MAX 70

// 0xRRGGBB. Cyan während der Messung, unabhängig von den Farben des Cores.
#define COLOR_MEASURE 0x005050

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = BATTERY_TX_DUTY_MS;

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
static volatile bool accelWoke = false;
static bool recalibrateDuty = false;
static uint16_t appliedPressureMbar = 0;
static bool sensorsStayOn = false;
void onWakeUp();

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

int combineAirQualityScore(bool hasHumidityMeasurement, int humidityScore, bool hasCo2Measurement, int co2Score) {
  if (hasHumidityMeasurement && hasCo2Measurement) {
    return humidityScore + co2Score;
  }
  if (hasHumidityMeasurement) {
    return humidityScore * 100 / HUM_SCORE_MAX;
  }
  if (hasCo2Measurement) {
    return co2Score * 100 / CO2_SCORE_MAX;
  }
  return 0;
}

void putUint16(uint8_t offset, uint16_t value) {
  appData[offset] = highByte(value);
  appData[offset + 1] = lowByte(value);
}

bool powerSensorsOn() {
  // The LoRaWAN stack may drive Vext during join, so the pin level is not our power state.
  bool needsColdStart = !sensorsStayOn;
  digitalWrite(Vext, LOW);
  if (needsColdStart) {
    // SCD30 accepts I2C commands about 2 s after power-up.
    delay(2000);
  }
  Wire.begin();
  return needsColdStart;
}

// Bus zu und Versorgung aus. Das nächste Einschalten ist ein Kaltstart.
void cutSensorPower() {
  Wire.end();
  digitalWrite(Vext, HIGH);
  LoRaWAN.holdVext(false);
  sensorsStayOn = false;
}

// Bus zu. keepPowered lässt die Versorgung für den nächsten warmen Zyklus an.
void parkSensors(bool keepPowered) {
  Wire.end();
  if (keepPowered) {
    digitalWrite(Vext, LOW);
    LoRaWAN.holdVext(true);
    sensorsStayOn = true;
    return;
  }
  digitalWrite(Vext, HIGH);
  LoRaWAN.holdVext(false);
  sensorsStayOn = false;
  appliedPressureMbar = 0;
}

// Pascals to mbar. 0 when there is no reading. Outside 700–1200 mbar the value is clamped.
uint16_t toPressureMbar(int32_t pressurePa, bool havePressure) {
  if (!havePressure) {
    return 0;
  }
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

bool connectScd30(uint16_t pressureMbar) {
  if (startScd30(pressureMbar)) {
    return true;
  }
  delay(1000);
  return startScd30(pressureMbar);
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

bool startScd30(uint16_t pressureMbar) {
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

  // Debug: die LED ist auch auf Batterie an, solange der Knoten wach ist.
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

  appTxDutyCycle = mainsConnected ? MAINS_TX_DUTY_MS : BATTERY_TX_DUTY_MS;
 
  Serial.printf("Measure uptime=%u battery=%u mV mains=%u interval=%lu s\r\n",
                uptimeCount, batteryVoltage, mainsConnected ? 1 : 0, (unsigned long)(appTxDutyCycle / 1000));
  
  // RGB and I2C share GPIO port 0. Lighting the LED writes the whole port, so do it before Wire.begin().
  if (LoRaWAN.isRgbEnabled()) {
    turnOnRGB(COLOR_MEASURE, 0);
  }
  bool needsColdStart = powerSensorsOn();
  bool restartMeasurement;
  for (int attempt = 0; attempt < 2; attempt++) {
    bmeStatus = 0;
    scdStatus = 0;
    if (initBme680(mainsConnected) && readBme680(temperature1, humidity1, pressure, gas, mainsConnected)) {
      bmeStatus = 1;
    }

    uint16_t pressureMbar = toPressureMbar(pressure, bmeStatus == 1);
    bool pressureChanged = hasPressureSignificantlyChanged(pressureMbar);

    restartMeasurement = needsColdStart || pressureChanged;
    bool scdReady = false;
    if (!needsColdStart) {
      scdReady = attachScd30();
    }
    if (restartMeasurement) {
      scdReady = connectScd30(pressureMbar);
    }
    if (scdReady && pressureMbar != 0) {
      appliedPressureMbar = pressureMbar;
    }
    if (scdReady && recalibrateDuty) {
      calibrateScd30();
    }
    if (scdReady) {
      scdStatus = readScd30(co2, temperature2, humidity2, restartMeasurement);
    }

    // A warm wake whose sensors do not answer means the rail dropped in sleep.
    if (needsColdStart || attempt == 1 || (bmeStatus == 1 && scdStatus != 0)) {
      break;
    }
    Serial.println(F("Vext im Schlaf aus, Kaltstart"));
    cutSensorPower();
    appliedPressureMbar = 0;
    needsColdStart = powerSensorsOn();
  }

  // Mains keeps the rail on so the next wake can continue the SCD30 measurement.
  parkSensors(mainsConnected);

  bool hasHumidityMeasurement = bmeStatus == 1 || scdStatus == 1;
  int humidityScore = scoreHumidity(bmeStatus, scdStatus, restartMeasurement, humidity1, humidity2);
  bool hasCo2Measurement = scdStatus == 1;
  int co2Score = hasCo2Measurement ? calculateCo2Score(co2) : 0;
  int airQualityScore = combineAirQualityScore(hasHumidityMeasurement, humidityScore, hasCo2Measurement, co2Score);
  
  Serial.printf("AQI humidity=%d/%d co2=%d/%d total=%d status=%u\r\n",
                humidityScore, HUM_SCORE_MAX, co2Score, CO2_SCORE_MAX, airQualityScore,
                bmeStatus * 10 + scdStatus);

  appDataSize = 24;
  putUint16(0, uptimeCount);
  putUint16(2, batteryVoltage);
  putUint16(4, temperature1);
  putUint16(6, temperature2);
  putUint16(8, round(humidity1 / 10));
  putUint16(10, round(humidity2 / 10));
  appData[12] = (uint32_t)pressure >> 24;
  appData[13] = (uint32_t)pressure >> 16;
  appData[14] = (uint32_t)pressure >> 8;
  appData[15] = (uint32_t)pressure & 0xFF;
  putUint16(16, round(gas / 10));
  putUint16(18, airQualityScore);
  putUint16(20, co2);
  appData[22] = bmeStatus * 10 + scdStatus;
  appData[23] = mainsConnected ? 1 : 0;

  Serial.printf("TX T1=%ld T2=%ld H1=%ld H2=%ld P=%ld Gas=%ld CO2=%ld AQI=%d Bat=%u St=%u Mains=%u\r\n",
                (long)temperature1, (long)temperature2, (long)humidity1, (long)humidity2,
                (long)pressure, (long)gas, (long)co2, airQualityScore, batteryVoltage,
                appData[22], appData[23]);

  uptimeCount++;
}

void prepareBeforeSleep() {
  if (mainsConnected) {
    // turnOffRGB clears the pixel. holdVext leaves the rail on.
    LoRaWAN.holdVext(true);
    LoRaWAN.enableRgb();
    turnOffRGB();
    Serial.println(F("Schlaf: Vext bleibt an"));
  } else {
    LoRaWAN.holdVext(false);
    LoRaWAN.enableRgb();
    turnOffRGB();
    digitalWrite(Vext, HIGH);
    Serial.println(F("Schlaf: Vext aus"));
  }
  delay(50);
}

void onWakeUp() {
  if (deviceState == DEVICE_STATE_SLEEP && digitalRead(WAKE_UP_PIN) == HIGH) {
    accelWoke = true;
  }
}

void showBootColors() {
  // Nur beim Start. 4 × 400 ms, danach aus. Der Join setzt seine eigenen Farben.
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

  accelWoke = false;
  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);
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
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE: {
      txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      Serial.printf("Sleep %d s\r\n", (int)(txDutyCycleTime / 1000));
      prepareBeforeSleep();
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP: {
      if (accelWoke) {
        Serial.println(F("Wakeup: send now"));
        LoRaWAN.txNextPacket();
        accelWoke = false;
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

  if (mcpsIndication->Port == 4 && mcpsIndication->BufferSize >= 2) {
    int newSleepTime = mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8);
    appTxDutyCycle = newSleepTime * 1000;
    saveDr();
    Serial.printf("Duty cycle %lu ms\r\n", (unsigned long)appTxDutyCycle);
  } else if (mcpsIndication->Port == 5) {
    recalibrateDuty = true;
    Serial.println(F("SCD30: calibrate next cycle"));
  } else if (mcpsIndication->Port == 9) {
    Serial.println(F("Reset"));
    HW_Reset(0);
  }
}
