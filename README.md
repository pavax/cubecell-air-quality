# CubeCell Luftqualität

Firmware für das Heltec CubeCell **HTCC-AB01** (EU868, OTAA, Port 2). Sie misst BME680 und SCD30 und sendet alle Werte in einem 24-Byte-Paket.

Netz oder Batterie entscheidet ein Spannungsteiler an **GPIO3**. Der Teiler liegt unten an Masse. Liegt am Knoten 3,3 V, ist Netz an (`HIGH`). Ohne VIN zieht der Teiler den Pin auf `LOW`.

## Ein Zyklus

1. Aufwachen, Batteriespannung lesen, `GPIO3` lesen.
2. Modus wählen: Intervall, Gas-Heizer, RGB.
3. LED setzen, bevor der I2C-Bus startet. Die RGB-LED liegt auf demselben GPIO-Port wie SDA und SCL. Ein späteres Setzen der Farbe würde den Bus überschreiben.
4. Sensoren messen, Paket senden.
5. Schlaf. Der Core lässt `Vext` im Schlaf aus, solange der Sketch nichts anderes wählt. Diese Firmware ruft am Netz `LoRaWAN.holdVext(true)` auf, auf Batterie `holdVext(false)`. Bei `true` treibt der Schlafpfad den Pin nach dem Umschalten der UART-Leitung wieder low, und `turnOffRGB()` schaltet die Versorgung nicht ab. Die LED wird vor dem Schlaf schwarz, weil sie an `Vext` hängt. Antwortet ein Sensor beim nächsten Aufwachen nicht, war die Spannung doch weg und der Zyklus startet kalt.

Der erste Start zählt als Kaltstart: `Vext` ist aus, der SCD30 wird neu gestartet, die erste SCD30-Probe und die erste BME680-Probe nach dem Einschalten werden verworfen. Beim Start zeigt die LED 1,6 Sekunden Rot, Gelb, Grün und Blau, danach aus. Während des Beitritts setzt der Core die Farben selbst.

Der User-Button weckt den Knoten sofort und schaltet die LED für genau diesen Zyklus ein. Danach gilt wieder der Modus aus `GPIO3`.

## Batterie

`GPIO3` ist `LOW`. Abstand zwischen zwei Sendungen: **30 Minuten**.

| | während der Messung | im Schlaf |
|---|---|---|
| `Vext` (Sensorversorgung, aktiv low) | an | aus |
| SCD30 | an, Messung wird jedes Mal neu gestartet | aus |
| BME680 | an, Gas-Heizer aus | aus |
| RGB | an (Debug, auch auf Batterie) | aus |

Die erste SCD30-Probe nach dem Start wird verworfen. Die Luftfeuchte für den AQI kommt in diesem Zyklus vom BME680.

## Netz

`GPIO3` ist `HIGH`. Abstand zwischen zwei Sendungen: **5 Minuten**.

| | während der Messung | im Schlaf |
|---|---|---|
| `Vext` | an | an |
| SCD30 | an, Messung läuft über den Schlaf weiter | an |
| BME680 | an, Gas-Heizer 320 °C / 150 ms, bis zu 5 Zyklen | an, Heizer aus |
| RGB | an | aus |

Die Druckkompensation des SCD30 kommt vom BME680 (700–1200 mbar). Beim BME680 wird immer die erste Probe verworfen, die letzte plausible bleibt. Beim SCD30 wird die erste Probe nur nach einem Kaltstart verworfen. Lief die Messung durch den Schlaf weiter, gehen beide Feuchten in den AQI ein.

Ein Downlink kann das Intervall nur bis zur nächsten Messung ändern. Danach setzt der Sketch es wieder aus `GPIO3`: 30 Minuten auf Batterie, 5 Minuten am Netz.

## Luftqualität

Der AQI ist eine ganze Zahl von 0 bis 100. Höher ist besser. Er setzt sich aus Feuchte und CO2 zusammen. Der Gaswiderstand geht nicht in die Punktzahl ein.

**Feuchte, höchstens 30 Punkte.** Von 30 bis 60 % gibt es die vollen 30 Punkte. Darunter fällt die Punktzahl linear bis 0 bei 0 % Feuchte, darüber linear bis 0 bei 100 %. `map()` rechnet mit ganzen Prozent, der Nachkommaanteil fällt vorher weg. Liefern beide Sensoren einen Wert und die SCD30-Messung lief weiter, zählt der Mittelwert. Nach einem Neustart der Messung zählt nur der BME680. Fehlt der BME680, zählt der SCD30.

**CO2, höchstens 70 Punkte.** Bis 800 ppm gibt es die vollen 70 Punkte, ab 2000 ppm 0. Dazwischen linear, wieder in ganzen ppm. Ohne gültige SCD30-Messung gibt es keinen CO2-Anteil.

Sind beide Anteile da, werden sie addiert. Fehlt einer, wird der andere auf 100 hochgerechnet: die Feuchte mit `Punkte * 100 / 30`, das CO2 mit `Punkte * 100 / 70`. Fehlen beide, ist der AQI 0.

Der Gaswert bleibt roh im Paket. Die beheizte Metalloxid-Schicht des BME680 senkt ihren Widerstand, wenn Rauch oder andere flüchtige Stoffe sie erreichen. CO2 misst sie nicht. Auf Batterie ist der Heizer aus und der Wert 0. Das Feld speichert Ohm geteilt durch 10 und reicht bis 655350 Ω. Darüber wird der gesendete Wert abgeschnitten.

## Paket

24 Bytes, Port 2, big-endian. Die Temperaturen sind vorzeichenbehaftet zu lesen. Die Beispielspalte ist eine Messung am Netz. Der AQI darin ist die Punktzahl, die diese Firmware für diese Sensorwerte schreibt: 48 % Feuchte liegen im vollen Band (30 Punkte), 1639 ppm CO2 ergeben 22 Punkte.

| Bytes | Wert | Beispiel | bedeutet |
|---|---|---|---|
| 0–1 | Zähler, uint16 | 2 | zweite Sendung |
| 2–3 | Batterie, uint16 mV | 4234 | 4,234 V |
| 4–5 | BME-Temperatur, int16, 0,01 °C | 2506 | 25,06 °C |
| 6–7 | SCD-Temperatur, int16, 0,01 °C | 2420 | 24,20 °C |
| 8–9 | BME-Feuchte, uint16, 0,01 % | 4846 | 48,46 %, gemessen 48,468 % |
| 10–11 | SCD-Feuchte, uint16, 0,01 % | 4781 | 47,81 %, gemessen 47,811 % |
| 12–15 | Druck, uint32 Pa | 95647 | 956,47 hPa |
| 16–17 | Gas, uint16, Ω/10 | 18664 | 186640 Ω |
| 18–19 | AQI, uint16 | 52 | 30 Punkte Feuchte plus 22 Punkte CO2 |
| 20–21 | CO2, uint16 ppm | 1639 | 1639 ppm |
| 22 | Status, uint8 | 11 | BME `1`, SCD `1` |
| 23 | Netz, uint8 | 1 | `VIN` an |

Byte 22 ist `bme * 10 + scd`. `1` ist gültig, `0` fehlt, `9` ist ein SCD30-Timeout. `11` heißt also beide gültig, `19` heißt BME gültig und SCD30 im Timeout.

## Downlink und Kalibrierung

| Port | Inhalt |
|---|---|
| 2 | Uplink, 24 Bytes |
| 4 | Intervall in Sekunden, uint16 big-endian, gilt bis zur nächsten Messung |
| 5 | SCD30 bei der nächsten Messung auf 400 ppm kalibrieren, nach 120 s Dauerbetrieb |
| 9 | Reset |

Dieselbe Kalibrierung startet `AT+Reclibrate=1`. Die automatische Selbstkalibrierung des SCD30 bleibt aus. Eine Kalibrierung ist nur in echter Frischluft sinnvoll.

## Patches am CubeCell-Core

Der Build nutzt den lokalen Klon von [pavax/CubeCell-Arduino](https://github.com/pavax/CubeCell-Arduino), Commit `8ceb8da`, unter `C:\Users\patri\AppData\Local\arduino-cli-cubecell\sketchbook\hardware\CubeCell\CubeCell-Arduino`. Die folgenden Änderungen sind dort nicht committet. Der Fork bringt `enableRgb` / `rgbIsEnabled` schon mit: `turnOnRGB` leuchtet nur, wenn die LED freigegeben ist.

| Datei | Änderung | Warum |
|---|---|---|
| `cores/asr650x/Wire/Wire.h` | `I2CTIMEOUT` von 10 auf 150 | Der SCD30 antwortet nicht innerhalb von 10 ms. Mit 150 ms kommen die Messwerte an. |
| `libraries/LoRa/src/LoRaWan_APP.cpp` | Wartezeit nach `Vext` low in `turnOnRGB` von 1 ms auf 5 ms | Dieselbe Wartezeit wie im Heltec-RGB-Beispiel. 1 ms reicht nicht, bis die LED-Versorgung steht. |
| `libraries/LoRa/src/LoRaWan_APP.h`, `LoRaWan_APP.cpp` | `LoRaWAN.holdVext(bool)`. Voreinstellung `false`: `Vext` bleibt im Schlaf aus. `turnOffRGB` schaltet die Versorgung nur ab, wenn der Halt nicht gesetzt ist | Der Sketch wählt es pro Zyklus. Join und Downlink rufen `turnOffRGB` auf und würden die Sensoren sonst abschalten. |
| `cores/asr650x/lora/system/low_power.c` | `driveHeldVext()` direkt vor und nach `CySysPmDeepSleep`, nur wenn `holdVext(true)` | Der Schlafpfad legt UART-TX (`P3_1`) auf analog. `Vext` ist der Nachbarpin `P3_2`. Nur mit Halt wird `P3_2` danach wieder low getrieben. Ohne Halt schreibt der Schlafpfad den Pin nicht an. |
