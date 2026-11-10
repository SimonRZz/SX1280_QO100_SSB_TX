# Übergabe: QO-100 ist am Mischerausgang vorhanden, der Si4732 findet es nicht

**Stand:** 2026-11-10, 22:30 UTC+1
**Repo:** `SimonRZz/SX1280_QO100_SSB_TX`, Branch `claude/review-upstream-changes-uky0w`
**Letzter Commit:** `a05e1ef`
**Vorgänger:** `HANDOVER_SSB.md` — das dort beschriebene Hängen ist gelöst, das
Dokument bleibt als Protokoll der Fehlersuche stehen.

---

## 1. Das Problem in einem Satz

Ein RTL-SDR am Mischerausgang empfängt die QO-100-Baken hörbar. Der Si4732 am
exakt selben Punkt findet sie nicht — in keinem Modus, in keiner Bandbreite.

---

## 2. Was inzwischen funktioniert

| | Nachweis |
|---|---|
| I2C, Reset, Bus-Modus | Adresse 0x11, `probe after reset: 1` |
| SSB-Patch | 15832 Bytes, 1979 Blöcke, **kein Block abgelehnt**, ID 0x9D29 |
| AM vollständig | Abstimmung, RSSI, SNR, AGC, Audio, Rundfunkempfang mit Musik |
| **SSB empfängt** | −60 dBm an Pin 8 → rssi 38, snr 9, hörbarer Ton |
| Si4732-Linearität | 100 dBµV → 99, linear bis 31 dBµV, ±2 dB über 70 dB |
| Audioweg | LOUT → ADC1 → PWM → Kopfhörer, mit Regelung und Begrenzer |
| Drehregler-Abstimmung | Display-Seite 4 |

---

## 3. Die SSB-Geschichte — zwei Irrwege, die nicht zu wiederholen sind

### 3.1 Das Hängen (gelöst)

Der Chip verriegelte nach jedem SSB-Tune: CTS blieb dauerhaft auf 0, jedes
antwortende Kommando lieferte Nullen, auch `GET_REV`. Ein Hardware-Reset
plus POWER_UP holte ihn zurück.

**Ursache:** `REFCLK_FREQ` steht nach POWER_UP per Vorgabe auf 32768 und
`REFCLK_PRESCALE` auf 1. Der Treiber korrigierte das erst **nach** dem
Patch-Upload. Der Chip verbrachte also den gesamten Upload und seine
Initialisierung mit einer falschen Vorstellung seines eigenen Takts.

Widerlegt wurden dabei: FAST-Bit, Wartezeiten, GET_INT_STATUS-Polling,
Transaktionsform, Antwortlänge, ANTCAP=0, „nur 0x42/0x43/0x47 betroffen",
Taktjitter, zweites POWER_UP. Details in `HANDOVER_SSB.md` §5.

### 3.2 Der zweite Irrweg: „XOSCEN=1 reicht"

`XOSCEN=1` beim **Patch-POWER_UP** (so macht es `patchPowerUp()` der
Referenzbibliothek) behebt das Hängen ebenfalls. Daraus wurde geschlossen, man
könne damit einen hohen, sauberen Referenztakt behalten.

**Das war falsch.** Mit 36 MHz / 1125 = 32000 Hz hängt der Chip zwar nicht
mehr, **empfängt aber nichts**: ein bei −60 dBm direkt an Pin 8 eingespeister
Träger lieferte in AM rssi 52 und in USB rssi **0** — bei richtiger
Seitenbandlage, 2200 Hz Bandbreite, Lautstärke am Anschlag.

### 3.3 Die tatsächliche Anforderung

**Der SSB-Patch ignoriert `REFCLK_FREQ` und verlangt 32,768 kHz.**

```
32768 / 32000 = 1,024
```

Mit 32000 Hz liegt jede Seitenbandabstimmung 2,4 % daneben — bei 19,6 MHz sind
das **470 kHz**, weit außerhalb jedes Filters.

Verifiziert durch einen einzigen Befehl: `rclk freq 32k`, sonst nichts
geändert → rssi springt von 0 auf 38, snr 9, Ton hörbar.

**Konsequenz: 32,768 kHz ist alternativlos, solange SSB gebraucht wird.**

---

## 4. Der Preis: ein Oberwellenkamm

Ein 32,768-kHz-**Rechteck** hat Oberwellen alle 32,768 kHz über die gesamte
Kurzwelle. Bei 19,6 MHz ist das die 598. Harmonische, die Nachbarn stehen bei
19595,3 und 19628,1 kHz.

| Referenztakt | Rauschgrenze im ZF-Band (AM, bw 4000) |
|---|---|
| 25 MHz oder 36 MHz | **2 bis 13 dBµV** |
| 32,768 kHz | **27 bis 45 dBµV** |

Der Kamm hebt den Boden um rund **30 dB** — und zwar genau in dem Band, in dem
gesucht wird. In SSB zeigt der Empfänger an den Bakenfrequenzen entsprechend
rssi 27 bis 37, und das ist ausschließlich Eigenstörung.

**Damit ist der Tiefpass auf der RCLK-Leitung kein Komfort mehr, sondern der
kritische Pfad.** Siehe §8.1.

---

## 5. Die Kette

```
QO-100 10489,5–10490,0 MHz
   │
Schüssel + Bullseye-LNB (LO 9750,004 MHz gemessen, +4,3 kHz)
   │   ACHTUNG: spezifizierter LNB-Ausgangsbereich ist 950–1950 MHz.
   │   QO-100 liegt unterhalb des Eingangsbereichs und erscheint bei
   │   739,5–740,0 MHz, in einer Ecke, für die der LNB nicht spezifiziert ist.
Triplexer (mit Bias-T)
   │
RMS-11X Diodenringmischer
   │   LO: TinySA HIGH output, 720,137 MHz, +7 dBm
   │       (Alternative: Pico CLK0, 144 MHz, 5. Oberwelle, nur −9 dBm)
   │   Mischverlust ca. 20 dB
ZF-Tiefpass
   │
Si4732-A10 Pin 8 (AMI)
```

**LO niemals auf 720.000 MHz.** Der ADF4351 des TinySA läuft dort im
Integer-N-Modus und erzeugt über Referenz-Nebenlinien ein Produkt bei exakt
20000 kHz an der ZF, das wie eine Bake aussieht (snr 16).

---

## 6. Alle Messwerte

### 6.1 Der Si4732 misst korrekt

TinySA direkt an Pin 8, AM, bw 4000:

| eingespeist | angezeigt |
|---|---|
| 100 dBµV | 99 |
| 92 | 92 |
| 82 | 82 |
| 72 | 72 |
| 62 | 61 |
| 52 | 50 |
| 42 | 38 |
| 32 | 31 |

Linear über 70 dB, alle Punkte ±2 dB.

### 6.2 Rauschgrenze

Mischer angeschlossen, LO aus, AM, bw 4000, Referenz 25/36 MHz: **ca. 15 dBµV**.

Daraus: 15 dBµV = −92 dBm in 4 kHz; thermisch in 4 kHz = −138 dBm →
**Rauschmaß rund 46 dB**.

### 6.3 Was der Mischer liefert

AM, bw 4000, Fenster 19795–19805 kHz (leeres Band), Referenz 25/36 MHz:

| | Pegel |
|---|---|
| LO ein (TinySA, +7 dBm) | 2 bis 13 dBµV |
| LO aus | 0 (Anzeigeboden) |

**Unterschied 2 bis 4 dB.** Das umgesetzte LNB-Rauschen liegt auf oder unter
dem Eigenrauschen des Empfängers. Hörbar ist der Unterschied nicht — erst
recht nicht mit eingeschalteter Audio-Regelung, die ihn wegregelt.

### 6.4 Eingespeister Träger, AM gegen SSB

19,609 MHz, −60 dBm (= 47 dBµV), direkt an Pin 8:

| Modus | Referenz | rssi |
|---|---|---|
| AM, bw 4000 | 36 MHz | **52** (snr 15) |
| USB, bw 2200, auf 19608 | 36 MHz | **0** |
| USB, bw 2200, auf 19608 | 32,768 kHz | **38** (snr 9), Ton hörbar |

Die mittlere Zeile ist der Beleg für §3.3. Die untere zeigt außerdem, dass SSB
rund 14 dB niedriger anzeigt als AM — beim Vergleich von Zahlen also immer den
Modus mitdenken.

### 6.5 Der RTL-SDR am selben Punkt

* QO-100-FT8 sichtbar bei 19,436 MHz (LO 720,100)
* **BPSK-Bake hörbar bei 19.358,4 kHz** (LO 720,137), gemessen als 19,440 MHz
  Mitte minus 81,6 kHz in SDRangel

Damit ist bewiesen: Schüssel, LNB, Triplexer, Mischer und ZF-Tiefpass
arbeiten. Das Signal liegt am Si4732-Eingang an.

### 6.6 Register, direkt ausgelesen

Mit `rx raw 4 13 <prop_hi> <prop_lo>` (GET_PROPERTY), Referenz damals 36 MHz:

| Register | gelesen | Sollwert |
|---|---|---|
| 0x0201 REFCLK_FREQ | 0x7D00 = 32000 | 32000 ✓ |
| 0x0202 REFCLK_PRESCALE | 0x0465 = 1125 | 1125 ✓ |
| 0x0101 SSB_MODE | 0x8011 | 0x8011 ✓ |

SSB_MODE zerlegt: AUDIOBW=1 (2,2 kHz), SBCUTFLT=1, AVC_DIVIDER=0, AVCEN=0,
SMUTESEL=0, DSP_AFCDIS=1 — identisch zu `setSSBConfig(bw,1,0,0,0,1)` der
Referenz. Die Konfiguration ist nachweislich korrekt; der Patch hält sich
trotzdem nicht an REFCLK_FREQ.

### 6.7 Antennen-Abstimmkondensator ohne Wirkung

AM, fester Pegel:

```
antcap 0    rssi 49   snr 21
antcap 1    rssi 49   snr 21
antcap 16   rssi 49   snr 21
antcap 126  rssi 49   snr 21
antcap 1024 rssi 47   snr 20
antcap 6143 rssi 36   snr 15
```

**Das ist ein Befund, kein Nullresultat** — siehe §7.2.

---

## 7. Warum der RTL-SDR es hört

### 7.1 Rauschmaß

Si4732 rund **46 dB**, RTL-SDR im Direct Sampling je nach Messung 20 bis 30 dB.
15 bis 25 dB Unterschied.

Eine frühere Behauptung in diesem Projekt, beide seien ähnlich unempfindlich
und der Unterschied liege nur in der Auflösungsbandbreite (10-Hz-FFT-Bin gegen
500-Hz-Filter), **trägt nicht**, sobald die Bake tatsächlich *gehört* wird,
also in rund 3 kHz demoduliert.

### 7.2 Der Eingang wird falsch betrieben

Der AM-Eingang des Si4732 ist **nicht für 50 Ω ausgelegt**. Er erwartet eine
hochohmige, **resonante** Quelle — Ferritantenne oder Stab —, bei der die
Antenne die Spannungsüberhöhung mitliefert. Angeschlossen ist breitbandig 50 Ω.

**Der Beleg steht in §6.7:** Der interne Varaktor soll eine Induktivität in
Resonanz bringen. Ohne Induktivität gibt es nichts abzustimmen, und deshalb
ändern antcap 0 bis 126 exakt gar nichts.

---

## 8. Maßnahmen, nach Dringlichkeit

### 8.1 Tiefpass auf der RCLK-Leitung — kritischer Pfad

```
SI5351 CLK2 ──[ 1 kΩ ]──┬── Si4732 Pin 13 (RCLK)
                         │
                     [ 2,2 nF ]
                         │
                        GND
```

Grenzfrequenz 72 kHz. Bei 32,768 kHz: −0,8 dB. Bei 19,6 MHz: **−49 dB**. Aus
43 dBµV Kamm werden damit unter 0.

Der Chip bekommt statt eines Rechtecks einen Sinus — genau das, was ein
32,768-kHz-Uhrenquarz geliefert hätte, für den dieser Eingang gebaut ist.

Bauteile vorhanden. **Ohne diesen Schritt ist jede Bakensuche sinnlos**, weil
der Kamm 30 dB über dem liegt, was gesucht wird.

### 8.2 Resonanter Eingang — nächstgrößter Hebel, kostenlos

```
Mischer-IF → Tiefpass → [ 22 pF ] ──┬── Si4732 Pin 8
                                     │
                                 [ 2,2 µH ]
                                     │
                                    GND
```

Bei 19,6 MHz: X_L = 271 Ω, Resonanz bei 30 pF. Der interne Varaktor reicht von
7 bis 590 pF (95 fF × ANTCAP + 7 pF), 30 pF entsprechen **antcap ≈ 242** —
mitten im Bereich. Streukapazität der Verdrahtung verschiebt das nach unten,
also breit durchfahren.

Parallelkreis bei Güte 50: R_p = 13 kΩ, gegen 50 Ω theoretisch 24 dB
Spannungsüberhöhung, realistisch **10 bis 20 dB**. Ohne Strom, ohne
Zusatzrauschen. Bandbreite rund 400 kHz, siebt den Rest gleich mit.

Erst die fertige 2,2-µH-Drossel probieren (Prinzipnachweis), dann selbst
wickeln: **T37-2 (rot), rund 50 Windungen 0,3 mm für 1 µH** → Resonanz bei
antcap ≈ 620, deutlich höhere Güte. Güte ist hier wörtlich die Verstärkung.

**Messung:** AM, bw 4000, feste Frequenz, `rx antcap` von 0 bis 1000
durchfahren und RSSI beobachten. **Bewegt sich überhaupt etwas, ist die
Diagnose bestätigt.**

### 8.3 ZF-Verstärker (Aufgabe #15)

Emitterstufe, alle Teile vorhanden. **S9018** (fT ≈ 1,1 GHz) oder 2N3904.

| Bauteil | von | nach |
|---|---|---|
| R1 100 Ω | VBUS (5 V) | Vcc |
| C1 100 µF | Vcc | GND |
| C2 100 nF | Vcc | GND |
| R2 18 kΩ | Vcc | Basis |
| R3 4,7 kΩ | Basis | GND |
| C3 10 nF | Eingang | Basis |
| R4 47 Ω | Eingang | GND |
| R5 470 Ω | Vcc | Kollektor |
| C4 10 nF | Kollektor | Si4732 Pin 8 |
| R6 33 Ω | Emitter | Knoten E2 |
| R7 47 Ω | E2 | GND |
| C5 100 nF | E2 | GND |

Arbeitspunkt: Vcc ≈ 4,6 V, Basis ≈ 0,94 V, Emitter ≈ 0,29 V, **Kollektor
2 bis 3,5 V** (wichtigster Prüfwert). Verstärkung ≈ 470/(33+7) ≈ 21 dB brutto,
nach dem 47-Ω-Abschluss **netto rund 15 dB**.

R4 ist nicht optional: der Diodenringmischer will sein IF-Tor mit 50 Ω
abgeschlossen sehen.

8.1, 8.2 und 8.3 schließen einander nicht aus.

### 8.4 Flankierend

* **LTE-800-Sperrfilter** (bestellt): Tiefpass 790 MHz. Lässt 739,5 MHz durch
  und schneidet das gesamte LNB-Nutzband ab 950 MHz weg. Hinter den Triplexer,
  auf der gleichspannungsfreien Seite. 75/50-Ω-Fehlanpassung kostet 0,2 dB.
* **LO-Pegel**: Der Pico-Zweig liefert −9 dBm statt +7 dBm. Mit dem TinySA als
  LO derzeit kein Engpass; dauerhaft wäre ein ADF4351-Modul die saubere Lösung
  (720 MHz direkt, Referenzeingang aus dem GPS-gebundenen SI5351).

---

## 9. Bakenfrequenzen, kalibriert

Aus der eigenen RTL-SDR-Messung (BPSK-Bake = 10489,750 MHz bei ZF 19.358,4 kHz,
LO 720,137 MHz). Der Abstand von 250 kHz ist durch den Transponder festgelegt.

| Bake | Downlink | ZF | Art |
|---|---|---|---|
| untere | 10489,500 | **19108** | CW-Träger |
| mittlere | 10489,750 | **19358** | 400 Bd BPSK |
| obere | 10490,000 | **19608** | CW-Träger |
| FT8 | 10489,540 | 19148 | |

**Die BPSK-Bake ist das falsche Suchziel**: kein Träger, Leistung über rund
800 Hz verteilt, klingt wie Rauschen, snr 0. Die beiden CW-Baken sind Träger.

Der TinySA-Quarz kann bei 720 MHz bis zu 7 kHz danebenliegen → ±10 kHz suchen.

**Seitenbandlage beim Hören:**

```
USB:  Abstimmfrequenz = Träger − (0,3 … 1,5 kHz)
LSB:  Abstimmfrequenz = Träger + (0,3 … 1,5 kHz)
AM:   Abstimmfrequenz = Träger
```

In AM suchen (Filter symmetrisch), in USB hören (rund 1 kHz unterhalb).

---

## 10. Messfallen

1. **AM und SSB haben verschiedene RSSI-Skalen** — SSB zeigt rund 14 dB
   niedriger. Vergleichswerte nur im selben Modus.
2. **19500 kHz trägt einen hausgemachten Störer** (rssi 60–65, reproduzierbar
   über Tage). Der Empfänger parkt dort zwischen Befehlen. 300 kHz daneben ist
   Stille. Nicht für Empfang halten.
3. **CLK2 (Referenz) und CLK0 (Mischer-LO) sind nach jedem Reset aus.** CLK2
   schalten `rx am`/`rx usb`/`rx scan` selbst ein, CLK0 nicht.
4. **`rx mon` ist nicht gespeichert** und nach jedem Start aus.
5. **`rx scan` stimmt danach nicht auf den Fund ab.**
6. **Nach einem Fehlversuch bleibt der Chip tot** — jede weitere Messung
   befragt denselben Zustand. Vor jedem Experiment `rx am`.
7. **Ein Scan in USB mit schmalem Filter ist für Träger fast blind**, weil das
   Seitenbandfilter nur nach oben reicht.
8. **Die Audio-AGC verdeckt kleine Pegelunterschiede** — für quantitative
   Hörvergleiche `rx mon agc 0`.
9. **Der Befehlsparser nahm bis `da5f2c7` nur sechs Wörter** und kürzte
   stillschweigend. `rx raw 4 13 00 02 01` ging als `13 00 02` hinaus, und zwei
   verschiedene Registerabfragen lieferten dieselbe Antwort. Behoben, lehnt
   jetzt ab statt zu kürzen.

---

## 11. Firmware

| Befehl | Zweck |
|---|---|
| `rx am` / `rx usb` / `rx lsb` | Modus; schaltet CLK2 mit ein |
| `rx freq <kHz>` | Abstimmung, Dezimalstellen erlaubt |
| `rx bw <Hz>` | AM 1000–6000, SSB 500–4000 |
| `rx antcap <0..6143>` | Varaktor; Vorgabe 1, 0 = Automatik |
| `rx mon [0\|1]`, `rx mon vol`, `rx mon agc` | Empfängerton auf den Kopfhörer |
| `rx scan <von> <bis> <schritt>` | Pegeltabelle, bis 512 Punkte |
| `rx diag` | Takte, Chip-ID, Patch, RSQ, letzter Fehler |
| `rx ssbtest [rev\|rsq\|tune]` | SSB-Hochlauf mit Statusabfrage nach jedem Schritt |
| `rx raw <n> <hex...>` | beliebiges Kommando, n Antwortbytes |
| `rx xoscpatch <0\|1>` | XOSCEN nur beim Patch-POWER_UP, Vorgabe 1 |
| `rclk [0\|1]`, `rclk freq <25\|36\|32k>`, `rclk drive <2..8>` | Referenztakt |
| `lo [0\|1]` | Mischer-LO aus dem Gerät |

Empfangsseite auf Display-Seite 4 (lange drücken): Drehregler stimmt ab,
Kacheln für Schrittweite, Modus, Bandbreite, Monitorpegel, Si4732-Pegel,
Audio-AGC.

**Takterzeugung:** PLL A = 884 MHz → CLK1 52 MHz (SX1280) **und** MS2 für
32,768 kHz (884 MHz / (210 + 12485/16384) / 128, exakt). PLL B = 864 MHz →
CLK0 144 MHz (Mischer-LO). 25 und 36 MHz kommen aus PLL B.

---

## 12. Vorgehen

1. **§8.1 bauen** — RC-Tiefpass auf RCLK. Ohne ihn ist alles Weitere wertlos.
   Kontrolle: AM, bw 4000, `rx scan 19590 19630 1`. Der Boden muss von 27–45
   auf unter 15 dBµV fallen.
2. **§8.2 bauen** — Induktivität an Pin 8, `rx antcap` von 0 bis 1000
   durchfahren. Bewegt sich die RSSI, ist H7.2 bestätigt.
3. Dieselben Fenster messen und **hören**: AM zum Suchen, USB rund 1 kHz
   unterhalb zum Hören, 19598–19618 und 19098–19118 mit 1 kHz Schritten.
4. Reicht es nicht, **§8.3** dazu.

**Erwartung mit §8.1 + §8.2:** Boden zurück auf ~15 dBµV, Signal um 10–20 dB
angehoben. Damit stünde die obere CW-Bake erstmals über der Nachweisgrenze.

**Und nicht vergessen:** Eine CW-Bake ist getastet. Ein Ton, der nach ein paar
Sekunden aufhört und wiederkommt, ist genau das, was gesucht wird.
