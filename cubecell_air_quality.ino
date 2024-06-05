#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "BME680.h"
//#include "Zanshin_BME680.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include <simple_logger.h>
#include "credentials.h"

BME680_Class BME680;

SCD30 airSensor;

#define BME_680_READINGS 8

#define SCD30_READINGS 8

#define SCD30_MEASUREMENT_INTERVAL 2  // s seconds [s]

#define SCD30_CALIBRATION_TIME 60  // 120 seconds [s]

#define MAX_READING_WAIT_TIME SCD30_READINGS *SCD30_MEASUREMENT_INTERVAL * 1000  // seconds [ms]

#define WAKE_UP_PIN USER_KEY

#define DEFAULT_LOG_LEVEL logger::Debug  // DEBUG: set to Debug for more logging statements or to None for no logs

#define GAS_LOWER_LIMIT 70000

#define GAS_UPPER_LIMIT 250000

#define HUM_REFERENCE 40

#define HUM_AQI_PERCENTAGE 0.14

#define GAS_AQI_PERCENTAGE 0.43

#define CO2_AQI_PERCENTAGE 0.43

#define CO2_LOWER_LIMIT 700

#define CO2_UPPER_LIMIT 2500

#define MAINS_CONNECTED_MV 4200

/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/




/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 1000 * 60 * 15;

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
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

static int32_t temperature1, temperature2, humidity1, humidity2, pressure, gas, co2;

static unsigned int batteryVoltage, uptimeCount;

bool mainsConnected = false;

bool accelWoke = false;

bool recalibrateDuty = false;

char buffer[20];

void blinkRGB(uint32_t color, int times = 3, int blinkTime = 500) {
  if (!LoRaWAN.isRgbEnabled()) {
    return;
  }
  for (int i = 0; i < times; i++) {
    turnOnRGB(color, blinkTime);
    turnOnRGB(0, blinkTime);
  }
}

float calcAltitude(const int32_t press, const float seaLevel = 1013.25);
float calcAltitude(const int32_t press, const float seaLevel) {
  /*!
    @brief     This converts a pressure measurement into a height in meters
    @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
    @param[in] press    Pressure reading from BME680
    @param[in] seaLevel Sea-Level pressure in millibars
    @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude = 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}

static void recalibrate() {
  logger::debug(F("SCD30: recalibrateDuty"));

  LoRaWAN.enableRgb();

  // wait some time while the sensor is running
  blinkRGB(0xff0000, SCD30_CALIBRATION_TIME, 500);  // blink red

  turnOnRGB(0x00FFFF, 0);  // cyan

  logger::debug(F("SCD30: setForcedRecalibrationFactor to %d"), 0);
  airSensor.setAltitudeCompensation(0);

  logger::debug(F("SCD30: setForcedRecalibrationFactor to %d"), 400);
  airSensor.setForcedRecalibrationFactor(400);

  logger::debug(F("SCD30: setAutoSelfCalibration to false"));
  airSensor.setAutoSelfCalibration(false);

  logger::debug(F("SCD30: setMeasurementInterval to %d"), SCD30_MEASUREMENT_INTERVAL);
  airSensor.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL);
}

static void prepareTxFrame(uint8_t port) {
  detachInterrupt(WAKE_UP_PIN);
  batteryVoltage = getBatteryVoltage();
  mainsConnected = batteryVoltage >= MAINS_CONNECTED_MV;
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

  if (digitalRead(Vext) == HIGH) {
    digitalWrite(Vext, LOW);
    delay(500);
  }
  blinkRGB(COLOR_SEND, 4, 250);  // blink red
  turnOnRGB(COLOR_SEND, 0);      // red

  logger::debug(F("Up-Time Count: %d"), uptimeCount);

  Wire.begin();
  delay(100);

  humidity1, humidity2, co2, temperature1, temperature2, pressure, gas = 0;

  char statusCode = 0x00;

  //
  // BME 680
  //
  logger::debug(F("BME680: init"));
  turnOnRGB(0xBCFF00, 500);  // green
  if (BME680.begin(I2C_STANDARD_MODE)) {
    logger::debug(F("BME680: - Setting 16x oversampling for all sensors"));
    BME680.setOversampling(TemperatureSensor, Oversample16);
    BME680.setOversampling(HumiditySensor, Oversample16);
    BME680.setOversampling(PressureSensor, Oversample16);
    logger::debug(F("BME680: - Setting IIR filter to a value of 4 samples"));
    BME680.setIIRFilter(IIR4);
    logger::debug(F("BME680: - Setting gas measurement to 320\xC2\xB0\C for 150ms"));
    BME680.setGas(320, 150);  // 320�c for 150 milliseconds

    turnOnRGB(0xCD00FF, 0);  // purple
    logger::debug(F("BME680: Start to measure"));

    for (int i = 0; i < BME_680_READINGS; i++) {
      //logger::debug(F("BME680: - Read Data"));
      BME680.getSensorData(temperature1, humidity1, pressure, gas, true);
      //Serial.print(gas / 100.0, 2);
      //Serial.println(F("mOhm"));
      if (gas != 0 && pressure != 0 && temperature1 != 0 && humidity1 != 0) {
        statusCode = (statusCode & 0x0F) | 0x10;  //status 1-X
      }
    }
    logger::debug(F("BME680: Done"));
  } else {
    logger::err(F("BME680: Not available"));
    statusCode = (statusCode & 0x0F);  // status 0-X
  }

  //
  // SCD 30
  //
  logger::debug(F("SCD30: init"));
  turnOnRGB(0x00FFFF, 500);  // cyan
  if (airSensor.begin()) {

    if (recalibrateDuty) {
      recalibrate();
      recalibrateDuty = false;
    }

    unsigned long startTime = millis();
    int readingCounter = 0;
    logger::debug(F("SCD30: Start to measure"));
    while (readingCounter < SCD30_READINGS) {
      // check if timed out
      if ((startTime + MAX_READING_WAIT_TIME) < millis()) {
        Serial.println("SCD30: - Error Timed out");
        statusCode = (statusCode & 0xF0) | 0x09;  // status X-9
        break;
      }

      if (airSensor.dataAvailable()) {
        logger::debug(F("SCD30: - Read Data"));
        co2 = airSensor.getCO2();
        temperature2 = airSensor.getTemperature() * 100;
        humidity2 = airSensor.getHumidity() * 1000;
        startTime = millis();
        readingCounter++;
        statusCode = (statusCode & 0xF0) | 0x01;  // status X-1
        // wait a bit if it is not the laster iteration
        if (readingCounter < SCD30_READINGS) {
          if (LoRaWAN.isRgbEnabled()) {
            turnOnRGB(COLOR_RECEIVED, 1000);
            turnOnRGB(0, 1000);
          } else {
            delay(2000);
          }
        }
      } else {
        if (LoRaWAN.isRgbEnabled()) {
          turnOnRGB(COLOR_JOINED, 250);
          turnOnRGB(0, 250);
        } else {
          delay(50);
        }
      }
    }
    logger::debug(F("SCD30: Done"));
  } else {
    statusCode = (statusCode & 0xF0);  // status X-0
    logger::err(F("SCD30: Not available"));
  }

  turnOnRGB(COLOR_RXWINDOW1, 0);
  Wire.end();
  digitalWrite(Vext, HIGH);

  logger::debug(F("Temperature1(°C):  %d"), int(temperature1 / 100.0));
  logger::debug(F("Temperature2(°C):  %d"), int(temperature2 / 100.0));

  logger::debug(F("Hum1(%%):  %d"), int(humidity1 / 1000.0));
  logger::debug(F("Hum2(%%):  %d"), int(humidity2 / 1000.0));

  logger::debug(F("Pres(hpa):  %d"), int(pressure / 100.0));
  logger::debug(F("Altitude(m): %d"), (int)calcAltitude(pressure));

  logger::debug(F("Gas(mOhm):  %d"), int(gas / 100.0));
  logger::debug(F("Co2: (ppm):  %d"), co2);
  logger::debug(F("Battery: (mV):  %d"), batteryVoltage);

  int humValue = round((humidity1 + humidity2) / 2);
  int humidity_score = calculateHumidityScore(humValue / 1000.0);
  int gas_score = calculateGasScore(gas);
  int co2_score = calculateCo2Score(co2);
  // Combine results for the final IAQ index value (0-100% where 100% is good quality air)
  int air_quality_score = humidity_score + gas_score + co2_score;

  logger::debug(F("Air Quality: %d%% derived from 14%% of Humidity reading and 43%% of Gas reading and 43%% of Co2 reading - 100%% is good quality air"), air_quality_score);
  logger::debug(F("Humidity element was: %d%% of 14%%"), humidity_score);
  logger::debug(F("Gas element was: %d%% of 43%%"), gas_score);
  logger::debug(F("Co2 element was: %d%% of 43%%"), co2_score);
  logger::debug(F("Status-Code: %02X"), statusCode);

  appDataSize = 24;

  appData[0] = highByte(uptimeCount);
  appData[1] = lowByte(uptimeCount);

  appData[2] = highByte(batteryVoltage);
  appData[3] = lowByte(batteryVoltage);

  appData[4] = highByte(temperature1);
  appData[5] = lowByte(temperature1);

  appData[6] = highByte(temperature2);
  appData[7] = lowByte(temperature2);

  int tmp = round(humidity1 / 10);
  appData[8] = highByte(tmp);
  appData[9] = lowByte(tmp);

  tmp = round(humidity2 / 10);
  appData[10] = highByte(tmp);
  appData[11] = lowByte(tmp);

  tmp = pressure;
  appData[12] = tmp >> 24;
  appData[13] = tmp >> 16;
  appData[14] = tmp >> 8;
  appData[15] = tmp & 0xFF;

  tmp = round(gas / 10);
  appData[16] = highByte(tmp);
  appData[17] = lowByte(tmp);

  appData[18] = highByte(air_quality_score);
  appData[19] = lowByte(air_quality_score);

  appData[20] = highByte(co2);
  appData[21] = lowByte(co2);

  appData[22] = ((statusCode & 0xF0) >> 4) * 10 + (statusCode & 0x0F);

  appData[23] = mainsConnected ? 1 : 0;

  // for (size_t i = 0; i < appDataSize; ++i) {
  //   sprintf(buffer, "0x%02X ", appData[i]);
  //   Serial.print(buffer);
  // }
  // Serial.println();

  uptimeCount++;
}

void prepareBeforeSleep() {
  if (!isTxConfirmed) {
    LoRaWAN.disableRgb();
    digitalWrite(Vext, HIGH);
  }
  logger::set_level(DEFAULT_LOG_LEVEL);
  delay(50);
}

void onWakeUp() {
  if (deviceState == DEVICE_STATE_SLEEP && digitalRead(WAKE_UP_PIN) == HIGH) {
    Serial.println(F("Woke up by WAKE_UP_PIN during sleep"));
    accelWoke = true;
    delay(10);
  }
}

void initManualRun() {
  logger::set_level(logger::Debug);
  LoRaWAN.enableRgb();
  turnOnRGB(0x005050, 500);
  turnOnRGB(0x002450, 500);
  turnOnRGB(0x000050, 500);
  turnOffRGB();
}

void setup() {
  Serial.begin(115200);
  logger::set_serial(Serial);

  pinMode(Vext, OUTPUT);

  accelWoke = false;
  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

#if (AT_SUPPORT)
  enableAt();
#endif

  initManualRun();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
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
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        logger::debug("Go to sleep for: %d sec", (int)(txDutyCycleTime / 1000.0));
        prepareBeforeSleep();
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (accelWoke) {
          initManualRun();
          logger::debug(F("Start Sending Cylcle due to wakeup"));
          LoRaWAN.txNextPacket();
          accelWoke = false;

        } else {
          if (isTxConfirmed && LoRaWAN.hasReceivedAck()) {
            LoRaWAN.disableRgb();
            LoRaWAN.resetReceivedAck();
          }
          LoRaWAN.sleep();
        }
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}


int calculateHumidityScore(float current_humidity) {
  int lowerLimit = (HUM_REFERENCE - 2);
  int upperLimit = (HUM_REFERENCE + 2);

  // Humidity +/-5% around optimum
  if (current_humidity >= lowerLimit && current_humidity <= upperLimit) {
    return HUM_AQI_PERCENTAGE * 100;
  }

  if (current_humidity < lowerLimit) {
    return map(current_humidity, 0, lowerLimit, 0, HUM_AQI_PERCENTAGE * 100);
  } else {
    return map(current_humidity, upperLimit, 100, HUM_AQI_PERCENTAGE * 100, 0);
  }
}


int calculateGasScore(float gas_reference) {
  if (gas_reference <= GAS_LOWER_LIMIT) {
    return 0;
  } else if (gas_reference >= GAS_UPPER_LIMIT) {
    return GAS_AQI_PERCENTAGE * 100;
  }

  return map(gas_reference, GAS_LOWER_LIMIT, GAS_UPPER_LIMIT, 0, GAS_AQI_PERCENTAGE * 100);

  //Calculate gas contribution to IAQ index
  // float gas_score = (0.43 / (GAS_UPPER_LIMIT - GAS_LOWER_LIMIT) * gas_reference - (GAS_LOWER_LIMIT * (0.43 / (GAS_UPPER_LIMIT - GAS_LOWER_LIMIT)))) * 100.00;
  // if (gas_score > 43) gas_score = 43;  // Sometimes gas readings can go outside of expected scale maximum
  // if (gas_score < 0) gas_score = 0;    // Sometimes gas readings can go outside of expected scale minimum
  // return gas_score;
}


int calculateCo2Score(float co2_reference) {
  if (co2_reference <= CO2_LOWER_LIMIT) {
    return CO2_AQI_PERCENTAGE * 100;
  } else if (co2_reference >= CO2_UPPER_LIMIT) {
    return 0;
  }

  return map(co2_reference, CO2_LOWER_LIMIT, CO2_UPPER_LIMIT, CO2_AQI_PERCENTAGE * 100, 0);
}


//AT Command                Value
//+LORAWAN=1                LoRaWAN  1, LoRa 0
//+OTAA=1                   OTAA -1, ABP-0
//+Class=A                  Class A or C
//+ADR=1                    1 on 0 for off
//+IsTxConfirmed=1          LoRaWAN ACK Message 1 on, 0 off.
//+AppPort=2                The Application Port 2 for general APPs and 10 for TTN MAPPER.
//+DutyCycle=60000          The time between transmission in mS. Typically, 15000 to 3600000
//+ConfirmedNbTrials=8      The number of adaptive rate changes allowed.
//+DevEui=???               Unique (OTAA Mode)
//+AppEui=???               Unique (OTAA Mode)
//+AppKey=???               Unique (OTAA Mode)
//+NwkSKey=???              Unique (ABP Mode)
//+Passkey=???              Unique (ABP Mode)
//+DevAddr=???              Unique (ABP Mode)
//+LPM=1                    Low Power Mode
//+ChipID=?                 get ChipID
//+JOIN=1                   start join
//+DelCDKEY=1               to delete the CDKEY
//+DefaultSet=1             to reset parameter to Default setting
//AT+Reclibrate=1           reclibrate sensors next duty cycle
bool checkUserAt(char *cmd, char *content) {
  if (strcmp(cmd, "Reclibrate") == 0) {
    recalibrateDuty = true;
    Serial.println(F("recalibrate the next time "));
    return true;
  }
  return false;
}


void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  Serial.printf("+REV DATA: %s, RXSIZE: %d, PORT: %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA: ");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();

  if (mcpsIndication->Port == 4) {
    int newSleepTime = mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8);
    appTxDutyCycle = newSleepTime * 1000;
    saveDr();
    Serial.print(F("new DutyCycle received: "));
    Serial.print(appTxDutyCycle);
    Serial.println(F("ms"));
    //txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
    //LoRaWAN.cycle(txDutyCycleTime);
  } else if (mcpsIndication->Port == 5) {
    recalibrateDuty = true;
    Serial.println(F("recalibrate the next time "));
  } else if (mcpsIndication->Port == 9) {
    Serial.println(F("Reset"));
    HW_Reset(0);
  }
}
