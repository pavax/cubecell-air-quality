# CubeCell Luftqualität

Firmware für das Heltec CubeCell **HTCC-AB01** (EU868, OTAA, Port 2). Sie misst BME680 und SCD30 und sendet alle Werte in einem 26-Byte-Paket.

Ob der Knoten im **Netzbetrieb** oder im **Batteriebetrieb** läuft, erkennt die Firmware über einen Spannungsteiler an Pin GPIO3. Am Teiler liegt **VIN** an. Liegt dort Spannung an, gilt Netzbetrieb. Ohne externe Versorgung zieht der Teiler den Pin auf Masse, und es gilt Batteriebetrieb.

## Ein Zyklus

1. Aufwachen, Batteriespannung lesen, Betriebsart erkennen.
2. Modus wählen: Intervall, Gas-Heizer, RGB.
3. Mess-LED setzen, bevor die Sensoren angesprochen werden. Die RGB-LED teilt sich den Port mit dem Sensorbus; die Farbe muss deshalb vorher stehen.
4. Sensoren messen und das Sendfenster aktualisieren (Mittelwerte/Extrema). Uplink nur, wenn das Sendintervall fällig ist, der User-Button gedrückt wurde oder es der erste Zyklus nach Start/Join ist; sonst nur messen. Im Netzbetrieb bleiben AQI-LED und Gas-Baseline mit jedem Messzyklus aktuell.
5. Schlaf. Im Netzbetrieb bleibt die Sensorversorgung an, und die RGB-LED zeigt die AQI-Farbe erst nach den LoRaWAN-Statusfarben — aber nur wenn mindestens ein Sensor gültig geantwortet hat. Sind beide tot, bleibt die LED aus und die Versorgung wird ausgeschaltet, damit der nächste Zyklus kalt startet. Im Batteriebetrieb sind LED und Sensorversorgung aus.

Der erste Start zählt als Kaltstart: die Sensorversorgung war aus, der SCD30 wird neu gestartet, die erste SCD30-Messung und die erste BME680-Messung nach dem Einschalten werden verworfen. Beim Start zeigt die LED kurz Rot, Gelb, Grün und Blau, danach aus. Während des LoRaWAN-Beitritts setzt die Plattform die Farben selbst.

Der User-Button weckt den Knoten sofort, erzwingt einen Uplink und schaltet die LED für genau diesen Zyklus ein. Danach gilt wieder die erkannte Betriebsart.

Ein Soft-Watchdog wird zu Beginn jedes Messzyklus (`prepareTxFrame`) auf **6 Stunden** gesetzt. Läuft in der Zeit kein neuer Zyklus an — z. B. weil die Messung hängt — setzt die Firmware den Knoten zurück (`HW_Reset`). Er hängt amselben RTC-Timer wie der Sendetakt und hilft daher nicht, wenn der MCU im Deep Sleep steckt und gar nicht mehr aufwacht.

## Batteriebetrieb

Zwei Intervalle: **messen** (Default **1 Stunde**) und **senden** (Default **4 Stunden**). Zwischen den Uplinks werden die Messungen im Fenster aggregiert.

| | während der Messung | im Schlaf |
|---|---|---|
| Sensorversorgung | an | aus |
| SCD30 | an, Messung wird jedes Mal neu gestartet | aus |
| BME680 | an, Gas-Heizer aus | aus |
| RGB | an | aus |

Die erste SCD30-Messung nach dem Start wird verworfen. Die Luftfeuchtigkeit für den AQI kommt in diesem Zyklus vom BME680.

## Netzbetrieb

Zwei Intervalle: **messen** (Sleep-Takt, Default **5 Minuten**) und **senden** (Default **1 Stunde**). Zwischen den Uplinks aggregiert die Firmware das Messfenster; die AQI-LED und die Gas-Baseline laufen mit jedem Messzyklus weiter.

| | während der Messung | im Schlaf |
|---|---|---|
| Sensorversorgung | an | an, aus wenn beide Sensoren tot |
| SCD30 | an, Messung läuft über den Schlaf weiter | an, aus wenn beide tot |
| BME680 | an, Gas-Heizer 320 °C / 150 ms, bis zu 5 Zyklen | an (Heizer aus), aus wenn beide tot |
| RGB | an | AQI-Farbe, aus ohne Messung |

Die Druckkompensation des SCD30 kommt vom BME680 (700–1200 mbar). Beim BME680 wird immer die erste Messung verworfen, die letzte plausible bleibt. Beim SCD30 wird die erste Messung nur nach einem Kaltstart verworfen. Lief die Messung durch den Schlaf weiter, gehen beide Luftfeuchtigkeitswerte in den AQI ein.

Port 4 setzt die vier Intervalle dauerhaft (EEPROM). Ohne gespeicherte Werte: Batterie messen 1 h / senden 4 h; Netz messen 5 min / senden 1 h.

## Luftqualität

Der AQI ist eine ganze Zahl von 0 bis 100. Höher ist besser. Er setzt sich aus Luftfeuchtigkeit, CO2 und dem Gaswiderstand zusammen.

**Luftfeuchtigkeit, höchstens 25 Punkte.** Von 30 bis 60 % gibt es die vollen 25 Punkte. Darunter fällt die Punktzahl linear bis 0 bei 0 %, darüber linear bis 0 bei 100 %. Es zählen ganze Prozent. Liefern beide Sensoren einen Wert und die SCD30-Messung lief weiter, zählt der Mittelwert. Nach einem Neustart der Messung zählt nur der BME680. Fehlt der BME680, zählt der SCD30.

**CO2, höchstens 50 Punkte.** Bis 800 ppm gibt es die vollen 50 Punkte, ab 2000 ppm 0. Dazwischen linear, in ganzen ppm. Ohne gültige SCD30-Messung gibt es keinen CO2-Anteil.

**Gas, höchstens 25 Punkte.** Nur im Netzbetrieb mit Heizer. Der Score ist das Verhältnis aktueller Gaswiderstand zur Baseline (höherer Widerstand = bessere Luft). Liegt der Widerstand unter etwa 20 % der Baseline, gibt es 0 Punkte. Die Baseline lebt nur im RAM: nach Kaltstart oder Batteriebetrieb lernen die ersten drei gültigen Gas-Messungen das Maximum, ohne schon in den AQI einzufließen. Danach hebt sauberere Luft die Baseline langsam an. Im Batteriebetrieb ist der Heizer aus, der Rohwert 0, und es gibt keinen Gas-Anteil.

Sind alle vorhandenen Anteile da, werden sie addiert. Fehlt mindestens einer, wird die Summe der übrigen auf 100 hochgerechnet. Fehlen alle, ist der AQI 0.

Im Netzbetrieb zeigt die RGB-LED diesen Wert im Schlaf in drei Stufen: Rot bei 0–33, Gelb bei 34–66, Grün bei 67–100. Fehlt jeder Anteil, bleibt die LED aus, damit fehlende Daten nicht wie schlechte Luft aussehen. Im Batteriebetrieb bleibt sie aus.

Der Gaswert bleibt zusätzlich roh im Paket. Die beheizte Metalloxid-Schicht des BME680 senkt ihren Widerstand, wenn Rauch oder andere flüchtige Stoffe sie erreichen. CO2 misst sie nicht. Das Feld speichert Ohm geteilt durch 10 und reicht bis 655350 Ω. Darüber wird der gesendete Wert abgeschnitten.

## Paket

26 Bytes, Port 2, big-endian. Die Temperaturen sind vorzeichenbehaftet zu lesen. Seit dem letzten Uplink aggregiert die Firmware das Messfenster; nach dem Senden wird es geleert. Die Beispielspalte ist eine Messung im Netzbetrieb nach der Gas-Lernphase.

| Bytes | Wert | Fenster | Beispiel | bedeutet |
|---|---|---|---|---|
| 0–1 | Zähler | nur Uplink | 2 | zweite Sendung |
| 2–3 | Batterie, mV | letzter Wert | 4234 | 4,234 V |
| 4–5 | BME-Temperatur, 0,01 °C | Mittelwert | 2506 | 25,06 °C |
| 6–7 | SCD-Temperatur, 0,01 °C | Mittelwert | 2420 | 24,20 °C |
| 8–9 | BME-Luftfeuchtigkeit, 0,01 % | Zyklus mit schlechtestem Humidity-Score | 4846 | 48,46 % |
| 10–11 | SCD-Luftfeuchtigkeit, 0,01 % | derselbe Zyklus | 4781 | 47,81 % |
| 12–15 | Druck, Pa | Mittelwert | 95647 | 956,47 hPa |
| 16–17 | Gas, Ω/10 | Minimum | 18664 | 186640 Ω |
| 18–19 | AQI | Minimum | 66 | schlechtester Zyklen-Score im Fenster |
| 20–21 | CO2, ppm | Maximum | 1639 | 1639 ppm |
| 22 | Status | aktuell | 11 | BME und SCD beide gültig |
| 23 | Betriebsart | aktuell | 1 | Netzbetrieb |
| 24–25 | Gas-Baseline, Ω/10 | aktuell | 18664 | 186640 Ω |

Byte 22 ist `bme * 10 + scd`. `1` ist gültig, `0` fehlt, `9` ist ein SCD30-Timeout. `11` heißt also beide gültig, `19` heißt BME gültig und SCD30 im Timeout. Byte 23 ist `1` im Netzbetrieb und `0` im Batteriebetrieb.

Ein Beispiel-Decoder für The Things Stack / ChirpStack liegt unter [`examples/payload_decoder.js`](examples/payload_decoder.js) (`decodeUplink` / `Decoder`, plus optionales `encodeDownlink` für Port 4/5/9).

## Downlink und Kalibrierung

| Port | Inhalt | Beispiel (Hex) |
|---|---|---|
| 2 | Uplink, 26 Bytes | — |
| 4 | Intervalle in Sekunden (uint16, big-endian), persistent im EEPROM | siehe Beispiele unten |
| 5 | SCD30 bei der nächsten Messung auf 400 ppm kalibrieren, nach 120 s Dauerbetrieb | `01` (Byteinhalt egal) |
| 9 | Soft-Reset (`HW_Reset`) | `01` (Byteinhalt egal) |

Port 4, **8 Bytes:** `battery_mode_measure_interval_s` + `battery_mode_tx_interval_s` + `mains_mode_measure_interval_s` + `mains_mode_tx_interval_s`. Ist `tx` kleiner als `measure` (je Modus), wird es auf das Messintervall angehoben. `0` in einem Wert wird verworfen.

| Bedeutung | Hex |
|---|---|
| Batterie 1 h / 4 h, Netz 5 min / 1 h (Defaults) | `0E103840012C0E10` |
| Batterie 1 h / 1 h, Netz 5 min / 1 h | `0E100E10012C0E10` |
| Batterie 2 h / 4 h, Netz 10 min / 1 h | `1C20384002580E10` |
| Batterie 6 h / 6 h, Netz 30 min / 1 h | `5460546007080E10` |

Port 5 und 9 werten nur den Port aus; ein beliebiges Byte reicht. Dieselbe Kalibrierung startet über den AT-Befehl zur Neukalibrierung. Die automatische Selbstkalibrierung des SCD30 bleibt aus. Eine Kalibrierung ist nur in echter Frischluft sinnvoll.

## Patches am CubeCell-Core

Der Build nutzt [pavax/CubeCell-Arduino](https://github.com/pavax/CubeCell-Arduino) `926c29b`.

Dieser Stand macht vier Dinge:

- Das I2C-Timeout ist länger, sonst antwortet der SCD30 nicht.
- Nach dem Einschalten der Sensorversorgung wartet die RGB-Ansteuerung kurz.
- Im Netzbetrieb bleibt die Sensorversorgung im Tiefschlaf an, außer beide Sensoren waren tot. Die AQI-Farbe kommt in `onEnterLowPower` (nach den LoRaWAN-Statusfarben SEND/RX/ACK) und wird bei jedem Deep-Sleep-Eintritt erneut gesetzt, weil der MAC die LED dazwischen mit `turnOffRGB` löschen kann. `turnOnRGB` lässt das Power-up-`delay` weg, wenn Vext schon gehalten wird. Nach dem Aufwachen bleibt die Versorgung an.
- Die BME680-Warte-Schleife hat ein Timeout von 3 s. Ohne das blieb die Mess-LED dauerhaft Cyan, wenn der Sensor im Measuring-Bit hing. Fehlt oder timeoutet ein Sensor, folgen bis zu zwei Power-Cycle-Retries (Versorgung erst ~4 s, dann ~8 s aus, jeweils mit Kaltstart). Bleiben beide tot, bleibt die Versorgung bis zum nächsten Zyklus aus.
