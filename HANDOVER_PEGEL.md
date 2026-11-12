# Übergabe: Si4732-SSB empfängt, Störungen noch offen

**Stand:** 2026-11-12

**Repo/Branch:** `SimonRZz/SX1280_QO100_SSB_TX`, `claude/review-upstream-changes-uky0w`

Diese Fassung ersetzt die widersprüchlichen älteren Pegel- und Bakenannahmen.
Historische Versuche bleiben in der Git-Historie und in `HANDOVER_SSB.md`
nachlesbar; alte Erklärungen sind keine zusätzlich bestätigten Befunde.

## Nachgewiesener Empfang und Aufbau

AM-Rundfunk und SSB-Empfang eines direkt eingespeisten Trägers funktionieren.
Der erfolgreiche Direkttest verwendete TinySA → AMI Pin 8, 19,609 MHz,
USB auf 19,608 MHz, 2200 Hz; der Ton war besonders mit `rx bfo 500` klar.
Im späteren Ein-/Aus-Vergleich: TinySA aus **RSSI 25 / SNR 1**, ein
**RSSI 35 / SNR 25**. Beim Abschalten verschwand der Pfeifton.

Auch über den Mischer wurden Satellitensignale um 19358,5–19360 kHz gehört.
Die genaue CW-/BPSK-Zuordnung bleibt offen. Ein RTL-SDR hört am selben
Mischerausgang im Direct-Sampling-Modus ebenfalls die Satellitensignale.

```text
Schüssel → Bullseye-LNB → Triplexer/Bias-T → RMS-11X → Si4732 AMI Pin 8
                                               ↑
                       TinySA HIGH: 720,137 MHz, +7 dBm, unmodulierter LO
```

**Beim erfolgreichen Aufbau gab es keinen ZF-Tiefpass.** Der später besprochene
LTE-800-Filter ist ein eigener Versuch. Hinter dem Mischer muss sein Durchlass
bei 19–20 MHz gemessen werden; er ersetzt keinen ZF-Tiefpass gegen Durchschläge
bei 720/740 MHz. Der eigene 144-MHz-LO würde über seine fünfte Oberwelle
720 MHz liefern; der berichtete Pegel von −9 dBm ist nicht gleichwertig mit
dem TinySA-LO von +7 dBm. Bei exakt 720,000 MHz wurde früher ein verdächtiges
Produkt um 20000 kHz beobachtet; Vergleiche deshalb weiter bei 720,137 MHz.

## Referenztakt: Befund und offene Erklärung

Erfolgreich getestet: **Si5351 CLK2 = 32768 Hz → Si4732 RCLK Pin 13**,
Prescaler 1 (`rclk freq 32k`). Mit 36 MHz / 1125 = 32000 Hz blieb SSB
für den direkt eingespeisten Träger ohne nutzbaren Empfang, während AM
ein Signal anzeigte. Die genaue interne Ursache ist nicht isoliert.
„Der Patch ignoriert grundsätzlich REFCLK_FREQ“ bleibt eine Hypothese.
Auch XOSCEN oder das abgelehnte zweite POWER_UP allein sind nicht als Ursache
bewiesen; die ERR-Meldung trat ebenfalls bei funktionierendem Empfang auf.

Der SSB-Patch umfasst 15832 Bytes / 1979 Blöcke, Patch-ID `0x9D29`.
Patch-ID, CTS, Register-Readbacks und „tune ok“ allein beweisen keinen HF-Empfang.
Ein RC-Glied mit 1 kΩ / 2,2 nF an RCLK wurde versucht; Rauschen/Wobbeln blieben.
Ein gemessener 30-dB-Oberwellenkamm oder ein zwingender RC-Bedarf sind nicht belegt.

| Einstellung | CLK2 / Prescaler | PLL B | eigener Mischer-LO |
|---|---|---|---|
| `25` | 25 MHz / 800 | 864 MHz | möglich |
| `36` | 36 MHz / 1125 | 864 MHz | möglich |
| `32k` | 32768 Hz / 1, MS2 / 375, R / 64 | 786,432 MHz | aus |
| `32m` | 32,768 MHz / 1000 | 864 MHz | möglich |

Für `32m` ist hier kein erfolgreicher SSB-Empfang dokumentiert. Gleiche
rechnerische Referenzfrequenz beweist keinen gleichwertigen Betrieb.
PLL A liefert weiterhin 52 MHz für TX. Der vorgeschlagene CLK2-Umbau auf
PLL A gehört nicht zu dieser Änderung.

## Frequenz und Kalibrierung

`ZF = Downlink − LNB-LO − Mischer-LO`. Mit dem bisherigen LNB-Messwert
9750,0043 MHz und TinySA-LO 720,137 MHz ergibt sich:

| Bezug | Downlink [MHz] | physische ZF [kHz] | `rx freq` bei `cal=-4300` [kHz] |
|---|---:|---:|---:|
| untere CW-Planposition | 10489,500 | 19358,7 | 19363 |
| mittlere BPSK-Planposition | 10489,750 | 19608,7 | 19613 |
| obere CW-Planposition | 10490,000 | 19858,7 | 19863 |
| FT8-Abstimmfrequenz | 10489,540 | 19398,7 | 19403 |

Die Bakenzeilen sind Planpositionen, keine gemessenen Trägerfrequenzen.
Für hörbare CW in USB muss die Abstimmung unter dem tatsächlichen Träger liegen.
Die CW-Baken verwenden laut [AMSAT-DL-Bandplan](https://amsat-dl.org/en/p4-a-nb-transponder-bandplan-and-operating-guidelines/)
F1A mit 400 Hz Hub. 19358,5 und 19360 kHz liegen nur 1,5 kHz auseinander;
das identifiziert keine zwei um rund 250 kHz getrennten Baken. Die frühere
Tabelle mit angeblich kalibrierter BPSK bei 19358 und CW bei 19108/19608 kHz
ist nicht als Referenz zu verwenden. Später wurde auch BPSK bei 19360 vermutet.

Der Treiber addiert `rx cal` zur angeforderten Frequenz. Die um 4,3 kHz
höhere LNB-LO setzt Signale 4,3 kHz tiefer um: dazu passt `rx cal -4300`.
Diese gerätespezifische Vorgabe gilt für neue/zurückgesetzte Einstellungen;
**gültige Flash-Werte, auch 0, bleiben erhalten**. RTL-SDR-/TinySA-Fehler und
LNB-Temperatur können zur Ablage beitragen. Der Wert ist keine universelle
LNB-Konstante und kein Nachweis einer GPS-kalibrierten Empfangskette.

Die Korrektur nur einmal anwenden. Für 10489,540 MHz sind dies Alternativen:

```text
rx cal 0       + rx freq 19398.7
rx cal -4300   + rx freq 19403
```

Für einen bekannten, direkt eingespeisten ZF-Testträger `rx cal 0` verwenden.
Seit `3bb1596` setzt die Feinabstimmung den BFO auf `-Rest` und stellt ihn
auch nach dem Laden wieder her. Ein manuelles `rx bfo` ersetzt diesen Wert;
der nächste `rx freq` berechnet ihn neu. 500 Hz BFO sind keine allgemeine Vorgabe.

## Audio und verbleibende Störanteile

Gemeldeter Aufbau: LOUT → 10 µF → Mittelpunkt eines 10-kΩ/10-kΩ-Teilers
zwischen 3V3 und GND → 1 kΩ → GP27; 47 nF von GP27 nach GND.
Der Teiler soll etwa 1,65 V liefern. Der gemeldete Elko-Pluspol zeigt zum
Si4732. Das ist zu prüfen: [PU2CLR](https://pu2clr.github.io/SI4735/extras/schematic/#external-mute-circuit)
nennt etwa 0,7 V DC am Audioausgang. Tatsächliche DC-Spannungen messen;
Plus gehört zur höheren Seite. Eine falsche Polung ist noch keine bewiesene Störursache.

LOUT → Pico-ADC → DC-Abzug → optionaler CW-Filter → Audio-AGC/Begrenzer →
PWM GP12 → Kopfhörer. HF-AGC im Si4732 und Pico-Audio-AGC sind getrennt.

* Audio-AGC aus: Ton und Störungen schwächer; das belegt keine SNR-Verbesserung.
* Si4732-Mute: Rauschen schwächer, Pulsieren weiter hörbar.
* Monitorlautstärke 0: Pulsieren deutlich schwächer, ein Rest bleibt.
* Si4732 aus, Monitor an, Audio-AGC aus: Hauptpulsieren bleibt.
* Dabei über 19,4 s `underruns=84` unverändert, Ring 80/256 bzw. 83/256.
  Diese RX-Pufferunterläufe erklären das laufende Pulsieren nicht.
* `in_peak=0.078` bzw. `0.076`, Gain 1: bei 3,3 V Referenz etwa 125 mV
  Spitzenabweichung nach DC-Abzug, keine RMS-/Oszilloskopmessung.
* Berühren des Spannungsteilers verändert/verzieht den Ton deutlich.
  Einkopplung, Versorgung, Masse und Kontakte sind damit noch nicht getrennt.
* OLED- und Kopfhörerpulsieren wurden als unterschiedlich beschrieben.
  Das bestätigt und widerlegt für sich allein keine OLED-Einkopplung.

Ein früherer Überlagerungston verschwand zusammen mit der Bake bei LO aus.
Daneben ist ein lokaler Störanteil ohne TinySA/LNB nachgewiesen. Beide
Befunde dürfen nicht zu einer einzigen bewiesenen Quelle zusammengefasst werden.
Ein abgeschalteter Si4732 ist kein definierter niederohmiger ADC-Abschluss.

## Diagnose und Firmwarestand

Diese Revision vereinheitlicht die Kalibrierungsvorgabe und zeigt den
Frequenzzustand genauer. `!R` behält `hz` als angeforderte ZF und ergänzt
`tuner_khz`, `if_hz`, `actual_hz` und `bfo_override`.
`if_hz` ist die aus Tuner/BFO berechnete konfigurierte ZF; `actual_hz`
ist deren nominales Äquivalent nach Abzug von `cal`. `bfo_override=1`
zeigt eine manuelle SSB-BFO-Abweichung vom automatischen Plan.
In `rx diag` heißen die ausführlichen Felder `requested_if_hz`,
`configured_if_hz` und `actual_nominal_hz`.
**Alle sind Software-Konfigurationswerte, keine unabhängigen RF-Messungen.**
`version`/Start/Diagnose zeigen `git=` mit 12 Hexstellen und gegebenenfalls
`-dirty` für Änderungen an versionierten Dateien beim Build.

Für Vergleiche sind vorhanden: `rx mon agc 0|1`, `rx mon cw 0` bzw.
`rx mon cw 150 600`, `rx mon level`, `rx mon maxgain`, `rx mon hang`,
`rx mon poll 0|1`, `oled freeze 0|1`, `oled off|on`, `smps pwm 0|1`.
SMPS-PWM ist die Firmwarevorgabe; das beweist keine vollständige Störfreiheit.

`rx mon` ohne Zahl gibt Diagnose aus und löscht Spitzenwerte; `rx mon 1`
initialisiert auch den Unterlaufzähler neu. Für Zählervergleiche zweimal
`rx mon` ohne Zahl aufrufen. Nach Neustart ist der Monitor aus.
`rx mute 1` bleibt innerhalb derselben Laufzeit auch nach `rx usb` erhalten.
`rx freq` bei Empfänger aus speichert nur; RSSI/SNR −1 sind dann keine Pegelmessung.

Der alte CLI-Parser kürzte Raw-Befehle nach sechs Tokens ab. Erst nach der
Korrektur waren die Vier-Byte-GET_PROPERTY-Abfragen gültig. Danach wurden bei
36 MHz REFCLK_FREQ 32000, PRESCALE 1125 und SSB_MODE `0x8011` gelesen.
Das beweist keine bestimmte interne Taktverarbeitung. `0x47` ist auch in
SSB als AGC_STATUS nutzbar; die frühere AM-Beschränkung war falsch.

## Nicht als Ursache oder Voraussetzung belegt

46 dB Rauschmaß, 15–25 dB Empfindlichkeitsdefizit, ein 30-dB-Referenzkamm und
eine feste AM-/SSB-RSSI-Differenz wurden nicht belastbar nachgewiesen.
Auch ANTCAP-Versuche beweisen keinen defekten Eingang. Resonanzkreis und
ZF-Verstärker sind nicht als Voraussetzung des beobachteten Empfangs belegt;
die versprochene LC-Überhöhung berücksichtigte Kopplung/Belastung nicht ausreichend.
AVC `0x8011 ↔ 0x9011` brachte im Hörvergleich keinen Unterschied.
Ein Quarz ist nicht zwingend: externe-Referenz-SSB-Projekte existieren, etwa
[Kholia/Pico](https://github.com/kholia/ConsensusBasedTimeSync),
[PE0MGB V3.3](https://github.com/pe0mgb/SI4735-Radio-ESP32-Touchscreen-Arduino/blob/master/SI473X_2.8_TFT_V3.3/SI4735_2.8_TFT_SI5351_V3.3.ino)
und [PU2CLR](https://github.com/pu2clr/SI4735/blob/master/examples/SI47XX_11_NO_CRYSTAL/POC_LCD16x2/POC_LCD16x2.ino).
Das beweist nicht die Qualität dieses Takts oder Audiowegs.

Nach Build/Softwareprüfung folgt die Hardwareuntersuchung: definierten
ZF-Träger ein/aus vergleichen, einen definierten ADC-Eingang gegen den
LOUT-/Biasweg prüfen und Referenz, Display sowie Versorgung einzeln variieren.
Dabei tatsächliche Analogauskopplung, Taktbeschaltung und Layout anderer
DIY-Projekte berücksichtigen. Der Hardwareerfolg dieser Revision steht noch aus.
