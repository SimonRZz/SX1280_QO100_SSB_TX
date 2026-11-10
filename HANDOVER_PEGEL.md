# Übergabe: Si4732 hört QO-100 nicht, der RTL-SDR am selben Punkt schon

**Stand:** 2026-11-10, 18:45 UTC+1
**Repo:** `SimonRZz/SX1280_QO100_SSB_TX`, Branch `claude/review-upstream-changes-uky0w`
**Letzter Commit:** `7cbc24c`
**Vorgänger-Dokument:** `HANDOVER_SSB.md` — jenes Problem ist **gelöst**, siehe §2.

---

## 1. Das Problem

Am Ausgang des Mischers liegt das QO-100-Signal nachweislich an: ein RTL-SDR
an genau diesem Punkt empfängt FT8 und die BPSK-Bake hörbar. Der Si4732 am
selben Punkt sieht nichts — weder in AM noch in SSB, weder im Pegelscan noch
im Kopfhörer.

Es fehlen geschätzt **15 bis 25 dB**. Die Frage ist, woher sie kommen sollen
und warum der RTL-SDR sie nicht braucht.

---

## 2. Was bereits gelöst ist — nicht erneut aufrollen

**SSB funktioniert** (Commits `1b42d8d`, `7cbc24c`). Ursache war: `REFCLK_FREQ`
steht nach POWER_UP per Vorgabe auf 32768 und `REFCLK_PRESCALE` auf 1. Der
Treiber korrigierte das erst **nach** dem Patch-Upload, sodass der Chip den
gesamten Upload mit einer falschen Vorstellung seines eigenen Takts verbrachte
(25 MHz am Pin, 32,768 kHz angenommen). Die gepatchte DSP hing dann beim ersten
Tune. Kein Block wurde abgelehnt, die Patch-ID las sich korrekt zurück.

Zwei Wege reparieren es, beide verifiziert:
* **`XOSCEN=1` nur beim Patch-POWER_UP** (so macht es die Referenzbibliothek in
  `patchPowerUp()`) — kostet nichts, ist jetzt Voreinstellung
* echte **32,768 kHz** auf RCLK — funktioniert ebenfalls, **wird aber nicht
  benutzt**: ein 32,768-kHz-Rechteck hat Oberwellen alle 32,768 kHz über die
  gesamte Kurzwelle und hob die Rauschgrenze von 2–13 auf flache 43 dBµV

Aktuell: Referenz **36 MHz / 1125 = 32000 Hz**, ganzzahliger MultiSynth-Teiler
aus PLL B, 16 MHz von der ZF entfernt. `xoscen(patch)=1`, `xoscen(operating)=0`.

---

## 3. Die Kette

```
QO-100 10489,5–10490,0 MHz
   │
Schuessel + Bullseye-LNB (LO 9750,004 MHz gemessen, +4,3 kHz)
   │   Ausgang 739,5–740,0 MHz
   │   ACHTUNG: der spezifizierte Ausgangsbereich des LNB ist 950–1950 MHz.
   │   QO-100 liegt unterhalb des Eingangsbereichs und kommt in einer Randecke
   │   heraus, in der der LNB nicht spezifiziert ist.
Triplexer (mit Bias-T fuer die LNB-Speisung)
   │
RMS-11X Diodenringmischer
   │   LO-Tor: TinySA HIGH output, 720,137 MHz, +7 dBm
   │           (Alternative: Pico CLK0 144 MHz, 5. Oberwelle 720 MHz, nur -9 dBm)
   │   Mischverlust gemessen ca. 20 dB
   │
ZF-Tiefpass
   │
Si4732-A10 Pin 8 (AMI), ZF ca. 19,1–19,9 MHz
```

**LO-Frequenz NIE auf 720.000 MHz stellen.** Der ADF4351 des TinySA laeuft dort
im Integer-N-Modus und erzeugt ueber Referenz-Nebenlinien ein Mischprodukt bei
exakt 20000 kHz an der ZF, das wie eine Bake aussieht (snr 16). Krumme Frequenz
verwenden.

---

## 4. Alle Messwerte

### 4.1 Der Si4732 selbst ist in Ordnung

TinySA direkt an Pin 8 eingespeist, AM, bw 4000:

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

Linear ueber 70 dB, alle Punkte innerhalb ±2 dB. Der Chip misst korrekt.

### 4.2 Rauschgrenze des Si4732

Mischer angeschlossen, LO aus, AM, bw 4000: **ca. 15 dBµV**.

Daraus abgeleitet: 15 dBµV = −92 dBm in 4 kHz. Thermisches Rauschen in 4 kHz
ist −138 dBm. **Rauschmaß also rund 46 dB.**

### 4.3 Was der Mischer liefert

AM, bw 4000, Fenster 19795–19805 kHz (leeres Band):

| Bedingung | Pegel |
|---|---|
| LO ein (TinySA 720,137 MHz, +7 dBm) | 2 bis 13 dBµV |
| LO aus | 0 (Anzeigeboden) |
| LO ein, Referenz 36 MHz | 0 |

Der Unterschied zwischen LO ein und aus betraegt **2 bis 4 dB**. Das umgesetzte
LNB-Rauschen liegt also **auf oder unter** dem Eigenrauschen des Empfaengers.

### 4.4 Der RTL-SDR am selben Punkt

Mischer-ZF statt an den Si4732 an den RTL-SDR (Direct Sampling):

* QO-100-FT8 bei 19,436 MHz sichtbar (LO 720,100)
* **BPSK-Bake hoerbar bei 19.358,4 kHz** (LO 720,137) — gemessen als
  19,440 MHz Mitte minus 81,6 kHz Versatz in SDRangel

Damit ist bewiesen: Schuessel, LNB, Triplexer, Mischer und ZF-Tiefpass
arbeiten. Das Signal liegt am Si4732-Eingang an.

### 4.5 Antennen-Abstimmkondensator ohne jede Wirkung

AM, fester Pegel, `rx antcap` durchgefahren:

```
antcap 0    rssi 49   snr 21
antcap 1    rssi 49   snr 21
antcap 16   rssi 49   snr 21
antcap 126  rssi 49   snr 21
antcap 1024 rssi 47   snr 20
antcap 6143 rssi 36   snr 15
```

**Das ist ein Befund, kein Nullresultat** — siehe §6.

---

## 5. Die Bakenfrequenzen, kalibriert

Aus der eigenen RTL-SDR-Messung abgeleitet (BPSK-Bake = 10489,750 MHz liegt bei
ZF 19.358,4 kHz, LO 720,137 MHz). Der Abstand von 250 kHz ist durch den
Transponder festgelegt.

| Bake | Downlink | ZF bei LO 720,137 | Art |
|---|---|---|---|
| untere | 10489,500 | **19108** | CW-Traeger |
| mittlere | 10489,750 | **19358** | 400 Bd BPSK |
| obere | 10490,000 | **19608** | CW-Traeger |
| FT8 | 10489,540 | 19148 | |

**Die BPSK-Bake ist das falsche Suchziel**: sie hat keinen Traeger, verteilt
ihre Leistung ueber rund 800 Hz, klingt wie Rauschen und liefert snr 0. Die
beiden CW-Baken sind Traeger und erzeugen beim Durchstimmen einen Pfeifton.

---

## 6. Die beiden Erklaerungen

### H1 — Rauschmaß

Si4732 rund **46 dB**. RTL-SDR im Direct Sampling je nach Quelle 20 bis 30 dB.
Das sind 15 bis 25 dB Unterschied und erklaert den Befund vollstaendig.

Anmerkung: In einer frueheren Sitzung wurde behauptet, beide seien aehnlich
unempfindlich und der Unterschied liege nur in der Aufloesungsbandbreite
(10-Hz-FFT-Bin gegen 500-Hz-Filter). **Das traegt nicht**, sobald der Operator
die Bake tatsaechlich *hoert*, also in rund 3 kHz demoduliert.

### H2 — Der Eingang wird falsch betrieben

Der AM-Eingang des Si4732 ist **nicht fuer 50 Ω ausgelegt**. Er erwartet eine
hochohmige, **resonante** Quelle — Ferritantenne oder Stab —, bei der die
Antenne die Spannungsueberhoehung mitliefert. Angeschlossen ist stattdessen
eine breitbandige 50-Ω-Quelle.

**Der Beleg steht in §4.5**: Der interne Varaktor soll eine Induktivitaet in
Resonanz bringen. Ohne Induktivitaet gibt es nichts abzustimmen, und genau
deshalb aendern antcap 0 bis 126 exakt gar nichts.

---

## 7. Zwei Loesungsansaetze

### A — Resonanter Eingang (zuerst probieren, kostet nichts)

```
Mischer-IF → Tiefpass → [ 22 pF ] ──┬── Si4732 Pin 8
                                     │
                                 [ 2,2 µH ]
                                     │
                                    GND
```

Rechnung bei 19,6 MHz: X_L = 271 Ω, Resonanzkapazitaet 30 pF. Der interne
Varaktor reicht von 7 bis 590 pF (95 fF × ANTCAP + 7 pF), 30 pF entsprechen
**antcap ≈ 240** — mitten im Bereich.

Parallelkreis bei Guete 50: R_p = 13 kΩ. Gegen 50 Ω Quelle theoretisch 24 dB
Spannungsueberhoehung, realistisch **10 bis 20 dB**. Ohne Strom, ohne
zusaetzliches Rauschen. Bandbreite rund 400 kHz, siebt den Rest des LNB-Bands
gleich mit aus.

Vorhanden: 2,2 µH fertige Drossel. **Besser selbst wickeln**: T37-6 (gelb,
10–50 MHz), rund 30 Windungen 0,3 mm Kupferlackdraht ≈ 2,2 µH mit deutlich
hoeherer Guete — und Guete ist hier woertlich die Verstaerkung.

**Messung:**
```
rx am
rx bw 4000
rx freq 19608
rx antcap 100   → rx
rx antcap 150   → rx
rx antcap 200   → rx
rx antcap 240   → rx
rx antcap 300   → rx
rx antcap 400   → rx
```
Zeigt sich ein Maximum, ist der Ansatz bestaetigt — ohne Induktivitaet tat sich
nachweislich nichts.

### B — ZF-Verstaerker (Aufgabe #15)

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
2 bis 3,5 V** (wichtigster Pruefwert). Verstaerkung ≈ 470/(33+7) ≈ 21 dB brutto,
nach dem 47-Ω-Abschluss **netto rund 15 dB**.

R4 ist nicht optional: der Diodenringmischer will sein IF-Tor mit 50 Ω
abgeschlossen sehen.

A und B schliessen einander nicht aus.

### C — Flankierend

* **LTE-800-Sperrfilter** (bestellt): Tiefpass mit Grenzfrequenz 790 MHz. Laesst
  739,5 MHz durch und schneidet das gesamte LNB-Nutzband ab 950 MHz weg. Holt
  den Mischer aus der Kompression. Hinter den Triplexer setzen (gleichspannungs-
  freie Seite), 75/50-Ω-Fehlanpassung kostet 0,2 dB.
* **LO-Pegel**: Der Pico-Zweig liefert nur −9 dBm statt +7 dBm. Dafuer ist ein
  PGA-103+ oder ein ADF4351-Modul vorgesehen. Mit dem TinySA als LO ist das
  derzeit kein Engpass.

---

## 8. Messfallen

1. **AM und SSB haben unterschiedliche RSSI-Skalen.** Vergleichswerte nur im
   selben Modus. Die Referenzzahl 2–13 dBµV bei 19795–19805 ist **AM, bw 4000**.
2. **19500 kHz traegt einen hausgemachten Stoerer** (rssi 60–65, reproduzierbar
   ueber mehrere Sitzungen). Der Empfaenger parkt dort zwischen Befehlen. 300 kHz
   daneben ist Stille. Nicht fuer Empfang halten.
3. **CLK2 (Referenz) und CLK0 (Mischer-LO) sind nach jedem Reset aus.** CLK2
   wird von `rx am`/`rx usb`/`rx scan` selbst eingeschaltet, CLK0 nicht.
4. **`rx mon` ist nicht gespeichert** und nach jedem Start aus.
5. **`rx scan` stimmt danach nicht auf den Fund ab.**
6. **Hoeren schlaegt Messen**: ein Traeger ist im 500-Hz-SSB-Filter 10 bis 15 dB
   frueher hoerbar, als ihn die RSSI-Anzeige meldet.

---

## 9. Firmware-Stand

| Befehl | Zweck |
|---|---|
| `rx am` / `rx usb` / `rx lsb` | Modus; schaltet CLK2 mit ein |
| `rx freq <kHz>` | Abstimmung, Dezimalstellen erlaubt |
| `rx bw <Hz>` | AM 1000–6000, SSB 500–4000 |
| `rx antcap <0..6143>` | Varaktor; **Vorgabe 1**, 0 = Automatik |
| `rx mon [0\|1]`, `rx mon vol`, `rx mon agc` | Empfaengerton auf den Kopfhoerer |
| `rx scan <von> <bis> <schritt>` | Pegeltabelle, bis 512 Punkte |
| `rx diag` | Takte, Chip-ID, Patch, RSQ, letzter Fehler |
| `rx ssbtest [rev\|rsq\|tune]` | SSB-Hochlauf mit Statusabfrage nach jedem Schritt |
| `rx raw <n> <hex...>` | beliebiges Kommando, n Antwortbytes |
| `rx xoscpatch <0\|1>` | XOSCEN nur beim Patch-POWER_UP, **Vorgabe 1** |
| `rclk [0\|1]`, `rclk freq <25\|36\|32k>`, `rclk drive <2..8>` | Referenztakt |
| `lo [0\|1]` | Mischer-LO aus dem Geraet |

Empfangsseite auf Display-Seite 4 (lange druecken): Drehregler stimmt den
Empfaenger ab, Kacheln fuer Schrittweite, Modus, Bandbreite, Monitorpegel,
Si4732-Pegel, Audio-AGC.

---

## 10. Vorgehen

1. **A bauen** (Induktivitaet an Pin 8), antcap durchfahren. Zeigt sich ein
   Maximum, ist H2 bestaetigt und es koennen 10–20 dB kommen.
2. Dieselben drei Fenster messen: `rx scan 19590 19630 1`, `rx scan 19090 19130 1`,
   und in AM bw 4000 `rx scan 19795 19805 1` gegen die Referenzzahl 2–13 dBµV.
3. Reicht es nicht, **B dazu** (Emitterstufe, netto 15 dB).
4. Abschliessend **hoeren**: `rx usb`, `rx bw 500`, `rx mon 1`, Drehregler in
   100-Hz-Schritten ueber 19600–19616 und 19100–19116. Eine CW-Bake erkennt man
   daran, dass der Pfeifton beim Durchdrehen von oben herunterkommt, im
   Nullpunkt verschwindet und wieder ansteigt.

**Erwartung mit 15 dB:** Satellitenrauschen 17–28 dBµV gegen 15 dBµV
Rauschgrenze — dann steht es 2 bis 13 dB darueber und die oberen CW-Bake wird
im 500-Hz-Filter hoerbar.
