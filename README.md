# CubeCell Luftqualität

Firmware für das Heltec CubeCell **HTCC-AB01** (EU868, OTAA, Port 2). Sie misst BME680 und SCD30 und sendet alle Werte in einem 26-Byte-Paket.

Ob der Knoten im **Netzbetrieb** oder im **Batteriebetrieb** läuft, erkennt die Firmware über einen Spannungsteiler an Pin GPIO3. Am Teiler liegt **VIN** an. Liegt dort Spannung an, gilt Netzbetrieb. Ohne externe Versorgung zieht der Teiler den Pin auf Masse, und es gilt Batteriebetrieb.

## Ein Zyklus

1. Aufwachen, Batteriespannung lesen, Betriebsart erkennen.
2. Modus wählen: Intervall, Gas-Heizer, RGB.
3. Mess-LED setzen, bevor die Sensoren angesprochen werden. Die RGB-LED teilt sich den Port mit dem Sensorbus; die Farbe muss deshalb vorher stehen.
4. Sensoren messen, Paket senden.
5. Schlaf. Im Netzbetrieb bleibt die Sensorversorgung an, und die RGB-LED zeigt die AQI-Farbe. Ohne gültige Messung bleibt die LED aus, die Versorgung aber an. Im Batteriebetrieb sind LED und Sensorversorgung aus. Antwortet ein Sensor beim nächsten Aufwachen nicht, war die Spannung doch weg und der Zyklus startet kalt.

Der erste Start zählt als Kaltstart: die Sensorversorgung war aus, der SCD30 wird neu gestartet, die erste SCD30-Messung und die erste BME680-Messung nach dem Einschalten werden verworfen. Beim Start zeigt die LED kurz Rot, Gelb, Grün und Blau, danach aus. Während des LoRaWAN-Beitritts setzt die Plattform die Farben selbst.

Der User-Button weckt den Knoten sofort und schaltet die LED für genau diesen Zyklus ein. Danach gilt wieder die erkannte Betriebsart.

## Batteriebetrieb

Abstand zwischen zwei Sendungen: **30 Minuten**.

| | während der Messung | im Schlaf |
|---|---|---|
| Sensorversorgung | an | aus |
| SCD30 | an, Messung wird jedes Mal neu gestartet | aus |
| BME680 | an, Gas-Heizer aus | aus |
| RGB | an | aus |

Die erste SCD30-Messung nach dem Start wird verworfen. Die Luftfeuchtigkeit für den AQI kommt in diesem Zyklus vom BME680.

## Netzbetrieb

Abstand zwischen zwei Sendungen: **5 Minuten**.

| | während der Messung | im Schlaf |
|---|---|---|
| Sensorversorgung | an | an |
| SCD30 | an, Messung läuft über den Schlaf weiter | an |
| BME680 | an, Gas-Heizer 320 °C / 150 ms, bis zu 5 Zyklen | an, Heizer aus |
| RGB | an | AQI-Farbe, aus ohne Messung |

Die Druckkompensation des SCD30 kommt vom BME680 (700–1200 mbar). Beim BME680 wird immer die erste Messung verworfen, die letzte plausible bleibt. Beim SCD30 wird die erste Messung nur nach einem Kaltstart verworfen. Lief die Messung durch den Schlaf weiter, gehen beide Luftfeuchtigkeitswerte in den AQI ein.

Ein Downlink kann das Intervall nur bis zur nächsten Messung ändern. Danach setzt die Firmware es wieder aus der Betriebsart: 30 Minuten im Batteriebetrieb, 5 Minuten im Netzbetrieb.

## Luftqualität

Der AQI ist eine ganze Zahl von 0 bis 100. Höher ist besser. Er setzt sich aus Luftfeuchtigkeit, CO2 und dem Gaswiderstand zusammen.

**Luftfeuchtigkeit, höchstens 25 Punkte.** Von 30 bis 60 % gibt es die vollen 25 Punkte. Darunter fällt die Punktzahl linear bis 0 bei 0 %, darüber linear bis 0 bei 100 %. Es zählen ganze Prozent. Liefern beide Sensoren einen Wert und die SCD30-Messung lief weiter, zählt der Mittelwert. Nach einem Neustart der Messung zählt nur der BME680. Fehlt der BME680, zählt der SCD30.

**CO2, höchstens 50 Punkte.** Bis 800 ppm gibt es die vollen 50 Punkte, ab 2000 ppm 0. Dazwischen linear, in ganzen ppm. Ohne gültige SCD30-Messung gibt es keinen CO2-Anteil.

**Gas, höchstens 25 Punkte.** Nur im Netzbetrieb mit Heizer. Der Score ist das Verhältnis aktueller Gaswiderstand zur Baseline (höherer Widerstand = bessere Luft). Liegt der Widerstand unter etwa 20 % der Baseline, gibt es 0 Punkte. Die Baseline lebt nur im RAM: nach Kaltstart oder Batteriebetrieb lernen die ersten drei gültigen Gas-Messungen das Maximum, ohne schon in den AQI einzufließen. Danach hebt sauberere Luft die Baseline langsam an. Im Batteriebetrieb ist der Heizer aus, der Rohwert 0, und es gibt keinen Gas-Anteil.

Sind alle vorhandenen Anteile da, werden sie addiert. Fehlt mindestens einer, wird die Summe der übrigen auf 100 hochgerechnet. Fehlen alle, ist der AQI 0.

Im Netzbetrieb zeigt die RGB-LED diesen Wert im Schlaf in drei Stufen: Rot bei 0–33, Gelb bei 34–66, Grün bei 67–100. Fehlt jeder Anteil, bleibt die LED aus, damit fehlende Daten nicht wie schlechte Luft aussehen. Im Batteriebetrieb bleibt sie aus.

Der Gaswert bleibt zusätzlich roh im Paket. Die beheizte Metalloxid-Schicht des BME680 senkt ihren Widerstand, wenn Rauch oder andere flüchtige Stoffe sie erreichen. CO2 misst sie nicht. Das Feld speichert Ohm geteilt durch 10 und reicht bis 655350 Ω. Darüber wird der gesendete Wert abgeschnitten.

## Paket

26 Bytes, Port 2, big-endian. Die Temperaturen sind vorzeichenbehaftet zu lesen. Die Beispielspalte ist eine Messung im Netzbetrieb nach der Gas-Lernphase. Der AQI darin ist die Punktzahl für diese Sensorwerte: 48 % Luftfeuchtigkeit liegen im vollen Band (25 Punkte), 1639 ppm CO2 ergeben 16 Punkte, Gas auf Baseline-Niveau ergibt 25 Punkte.

| Bytes | Wert | Beispiel | bedeutet |
|---|---|---|---|
| 0–1 | Zähler | 2 | zweite Sendung |
| 2–3 | Batterie, mV | 4234 | 4,234 V |
| 4–5 | BME-Temperatur, 0,01 °C | 2506 | 25,06 °C |
| 6–7 | SCD-Temperatur, 0,01 °C | 2420 | 24,20 °C |
| 8–9 | BME-Luftfeuchtigkeit, 0,01 % | 4846 | 48,46 % |
| 10–11 | SCD-Luftfeuchtigkeit, 0,01 % | 4781 | 47,81 % |
| 12–15 | Druck, Pa | 95647 | 956,47 hPa |
| 16–17 | Gas, Ω/10 | 18664 | 186640 Ω |
| 18–19 | AQI | 66 | 25 Punkte Luftfeuchtigkeit plus 16 Punkte CO2 plus 25 Punkte Gas |
| 20–21 | CO2, ppm | 1639 | 1639 ppm |
| 22 | Status | 11 | BME und SCD beide gültig |
| 23 | Betriebsart | 1 | Netzbetrieb |
| 24–25 | Gas-Baseline, Ω/10 | 18664 | 186640 Ω |

Byte 22 ist `bme * 10 + scd`. `1` ist gültig, `0` fehlt, `9` ist ein SCD30-Timeout. `11` heißt also beide gültig, `19` heißt BME gültig und SCD30 im Timeout. Byte 23 ist `1` im Netzbetrieb und `0` im Batteriebetrieb.

## Downlink und Kalibrierung

| Port | Inhalt |
|---|---|
| 2 | Uplink, 26 Bytes |
| 4 | Intervall in Sekunden, gilt bis zur nächsten Messung |
| 5 | SCD30 bei der nächsten Messung auf 400 ppm kalibrieren, nach 120 s Dauerbetrieb |
| 9 | Reset |

Dieselbe Kalibrierung startet über den AT-Befehl zur Neukalibrierung. Die automatische Selbstkalibrierung des SCD30 bleibt aus. Eine Kalibrierung ist nur in echter Frischluft sinnvoll.

## Patches am CubeCell-Core

Der Build nutzt [pavax/CubeCell-Arduino](https://github.com/pavax/CubeCell-Arduino) `926c29b`.

Dieser Stand macht vier Dinge:

- Das I2C-Timeout ist länger, sonst antwortet der SCD30 nicht.
- Nach dem Einschalten der Sensorversorgung wartet die RGB-Ansteuerung kurz.
- Im Netzbetrieb bleibt die Sensorversorgung im Tiefschlaf an. Vor dem Schlafen wird die AQI-Farbe gesetzt. Nach dem Aufwachen bleibt die Versorgung an, ohne die Farbe erneut zu setzen.
- Die BME680-Warte-Schleife hat ein Timeout von 3 s. Ohne das blieb die Mess-LED dauerhaft Cyan, wenn der Sensor im Measuring-Bit hing. Nach einem BME-Fehler folgt ein Kaltstart-Retry.
