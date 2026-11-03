# Übergabe: QO-100-Empfangszweig für SX1280_QO100_SSB_TX

Stand 2026-11-03. Branch `claude/qo100-rx-path`, letzter Commit `1bb9d3a`.

Dieses Dokument ist für eine Übernahme durch einen anderen Assistenten gedacht.
Es enthält den Entwurf, die Messwerte, die ungeprüften Annahmen und — wichtig —
eine Liste der Fehler, die bereits gemacht wurden, damit sie sich nicht
wiederholen.

---

## 1. Worum es geht

Ein QO-100-Sender (Es'hail-2, 2,4 GHz Uplink) auf Basis Raspberry Pi Pico 2
(RP2350) + SX1280 + GPS-geführtem SI5351. Sendebetrieb funktioniert seit Version
2.2.0 und ist auf `master` gemerged: SSB, CW mit Keyer am Gerät, OLED,
Rotary-Encoder, Mikrofon, PTT über die CW-Paddles, Einstellungen im Flash.

**Das aktuelle Vorhaben ist der Empfangszweig**, damit das Gerät ohne Computer
sendet *und* empfängt. Zielkette:

```
Spiegel → Bullseye-LNB → 739,5 MHz → Triplexer → SAW-Filter
       → RMS-11X-Mischer → ZF 19,5 MHz → 3 dB → Tiefpass
       → Si4732 (SSB) → NF → Pico ADC1/GP27 → Kopfhörer

SI5351 CLK2 144 MHz → Hochpass → SPF5189Z → Mischer-LO (5. Oberwelle, 720 MHz)
```

---

## 2. WORAN WIR GERADE SCHEITERN

**Kurzfassung: Die Kette ist gliedweise bewiesen, aber wir haben noch nie den
Satelliten durch sie gehört. Die wahrscheinlichste Ursache ist diagnostiziert,
aber noch nicht verbaut und damit nicht bestätigt.**

### 2.1 Der aktuelle Hauptblocker: fehlender Tiefpass am IF

Jede Stufe funktioniert einzeln:

| Glied | Status |
|---|---|
| LNB, Antenne, Triplexer | **bewiesen** — Transponder bei 739,5 MHz gesehen (RTL-SDR v3, Normalmodus, 20 dB Dämpfung) |
| Hochpass im LO-Zweig | **bewiesen** — 34 dB bei 144 MHz, 0 dB bei 720 MHz |
| SPF5189Z | läuft, aber nur 10 dB statt 18,7 dB (siehe 2.3) |
| RMS-11X-Mischer | **bewiesen** — setzt eingespeiste 739,5 MHz auf 19,5 MHz um, ~20 dB Mischverlust |
| Empfänger am IF | **hier scheitert es** |

Diagnose: Ein Ringmischer lässt seinen Oszillator in den IF-Port durch, die
LO-IF-Isolation liegt typisch bei 20–30 dB.

```
LO am Mischer           −5 dBm
LO-IF-Isolation         20…30 dB
= 720-MHz-Leck am IF    −25 bis −35 dBm

Bake (geschätzt)        −84 dBm
Abstand                 49 bis 59 dB
```

Der RTL-SDR hat einen **8-Bit-ADC mit etwa 48 dB Dynamikbereich**. Das Leck
belegt den oberen Rand, die Bake fällt ins Quantisierungsrauschen. Das erklärt
beide Beobachtungen des Benutzers:

- Mit eingeschalteter AGC rastet sie auf das Leck, nichts anderes ist hörbar.
- Mit abgeschalteter AGC ist ein eingespeistes Testsignal bis **−35 dBm
  Generatorpegel** hörbar, also etwa −55 dBm am IF. Das ist **keine
  Empfindlichkeitsgrenze**, sondern eine vom Leck angehobene Rauschgrenze.

**Die Lösung ist entworfen, aber noch nicht gebaut.** Tiefpass aus vorhandenen
Teilen, dokumentiert in `WIRING.txt`:

```
Mischer IF ──[3 dB]──┬──(((( 120 nH ))))──┬──► Empfänger
                     │                    │
                   27 pF                27 pF
                     │                    │
                    GND                  GND
```

| Frequenz | Dämpfung |
|---|---|
| 19,5–20,0 MHz (ZF) | 0,0 dB |
| 144 MHz | 5 dB |
| 720 MHz (LO-Leck) | **46 dB** |
| 739,5 MHz (HF-Leck) | 47 dB |

Damit sinkt der nötige Dynamikbereich von 59 dB auf 13 dB. Variante mit je
27 pF ∥ 15 pF (= 42 pF) gibt 54 dB, gleicher Aufwand.

**Dämpfungsglied gehört VOR den Tiefpass**, damit der Mischer eine ohmsche Last
sieht.

**Nächster Schritt: Benutzer baut den Tiefpass, dann neu messen.**

### 2.2 Der Si4732 war noch nie angeschlossen

Der komplette Treiber (Stufe R1) ist geschrieben, baut sauber, ist aber **gegen
keine Hardware getestet**. Alles unter „Si4732" in diesem Dokument ist
Schreibtischarbeit.

Sobald der Tiefpass sitzt, ist der Si4732 ohnehin der bessere Empfänger: −110 dBm
Empfindlichkeit gegen die geschätzten −80 bis −84 dBm am IF, also 25–30 dB
Reserve statt einer Grenzwertmessung.

### 2.3 SPF5189Z liefert 10 dB statt 18,7 dB

Gemessen: −19 dBm rein, −9 dBm raus (später auf −5 dBm verbessert, Ursache der
Verbesserung unklar). Beide Werte bei 720 MHz am selben TinySA-Eingang mit
derselben Einstellung, die Zahl ist also belastbar.

**Das ist aber KEIN Blocker.** Durchgerechnet (Friis, LNB mit 45 dB Verstärkung
und 1 dB Rauschmaß):

| LO-Pegel | Mischverlust | am Si4732 | System-Rauschmaß |
|---|---|---|---|
| +7 dBm (Nennwert) | 7,1 dB | −72,6 dBm | 1,06 dB |
| −0,3 dBm | 11 dB | −76,5 dBm | 1,15 dB |
| **−5 dBm (Istzustand)** | **15 dB** | **−80,4 dBm** | **1,36 dB** |
| −9 dBm | 18 dB | −83,6 dBm | 1,73 dB |

Von −5 dBm bis zum Nennwert gewinnt man **0,3 dB Systemrauschmaß**. Der LNB setzt
mit 45–50 dB Verstärkung das Rauschmaß allein; ein unterversorgter Mischer senkt
Signal und Rauschen gleichermaßen. Kostet Pegel, kaum Empfindlichkeit.

Zum Vergleich: ohne Verstärker (−17 dBm LO) wären es 3,6 dB Systemrauschmaß. Dort
macht der SPF5189Z den Unterschied, die restlichen 12 dB sind Komfort.

Übersteuerung ist auch kein Thema: RF-Kompression bei −5 dBm LO etwa −8 dBm,
tatsächlich am RF-Port nach dem SAW-Filter etwa −30 dBm, also 22 dB Abstand.

Zu prüfen, falls jemand die 9 dB doch holen will, in dieser Reihenfolge:
1. **Versorgungsspannung am Modul unter Last.** 5 V / 90 mA soll. Hauptverdacht.
2. **Speisung über die HF-Ausgangsbuchse?** Manche LNA-Module werden Bias-Tee-artig
   über den Ausgang gespeist. Dann hätte die empfohlene Gleichspannungssperre die
   Versorgung abgeschnitten.
3. **Nachbau?** Der SPF5189Z ist seit März 2021 abgekündigt.

### 2.4 „waiting for clock" nach dem Flashen

Nach dem Flashen von 2.3.0-rx blieb der Sender beim Booten in „waiting for
clock" hängen, obwohl der Benutzer an der Hardware nichts geändert hatte.

**Abgesichert** (Commit `a67f948`): CLK2 startet jetzt AUS, erst `lo 1` schaltet
es ein. `gpsdo_init()` läuft vor dem Boot-Gate, das auf den gerasteten 52-MHz-Takt
wartet — der Empfangszweig darf den Sender nicht am Starten hindern. Das war ein
Konstruktionsfehler, unabhängig von der Ursache.

**Ursache noch offen.** Diagnoseweg: Konsole verbinden (läuft während der
Wartemeldung), `gpsdo` tippen, Feld `clk1=` ablesen:

| `clk1=` | Bedeutung |
|---|---|
| `fail` | SI5351 nicht am I2C gefunden |
| `los` | kein Signal am XA-Pin, 24 MHz vom GPS fehlen |
| `lol` | XA vorhanden, PLL A rastet nicht |
| `ok` | Takt in Ordnung |

Dann `lo 1`, nochmal `gpsdo`. Kippt es von `ok` auf `lol`, ist die Last auf CLK2
der Täter — drei Mechanismen denkbar: Gleichspannung aus dem Verstärkermodul in
den CLK2-Pin, Einbruch der 3,3-V-Schiene durch die zusätzlichen 8 mA (OLED,
MAX4466 und GPS hängen dort ebenfalls), oder 144 MHz koppeln über eine schlechte
Masse in die XA-Leitung mit den 24 MHz.

**Zweiter Kandidat, unabhängig vom Empfangszweig:** `CFG-TP5` ist flüchtig und
wird bei jedem Boot nach nur `GPSDO_BOOT_DELAY_MS = 300 ms` gesendet. Bootet das
GPS-Modul langsamer, greift die Konfiguration nicht und TIMEPULSE bleibt auf dem
Vorgabewert 1 Hz — dann sieht der SI5351 keinen brauchbaren Referenztakt. Das
würde sporadisch auftreten. Abhilfe: Konfiguration wiederholen, bis der SI5351
rastet, statt sie einmal blind zu senden.

**Ausgeschlossen:** der fehlende Satellitenfix. `freqPeriod` und `freqPeriodLock`
stehen im CFG-TP5-Paket beide auf 24 000 000, die 24 MHz kommen also auch ohne Fix.

### 2.5 Möglicher Spur bei 20,000 MHz — unbestätigt

```
PLL A = 24 MHz × (36 + 5/6) = 884 MHz
PLL B = 24 MHz × 36         = 864 MHz
                 Differenz  = 20,000 MHz
```

Das liegt exakt an der Oberkante des ZF-Bands (19,5–20,0 MHz). Der Benutzer
berichtete „einen Träger über der Bake", dessen Frequenz noch nicht abgefragt
wurde.

**Diagnose:** Frequenz des Trägers feststellen. `lo 0` eingeben — PLL B läuft
dann weiter, nur CLK2 ist abgeschaltet. Verschwindet der Träger, kommt er über
das CLK2-Kabel.

**Lösung, falls bestätigt:** PLL A von 884 auf **624 MHz** umstellen, Teiler 12
statt 17.

```
24 MHz × 26 = 624 MHz,  624 ÷ 12 = 52,000000 MHz
Differenz zu PLL B: 864 − 624 = 240 MHz → weit aus dem ZF-Band
Register: PLLA p1 = 128×26 − 512 = 2816 = 0x0B00
          MS1  p1 = 128×12 − 512 = 1024 = 0x0400
```

Zweiter Vorteil: PLL A wäre damit **ganzzahlig**. Bisher läuft sie fraktional,
was Jitter auf dem Takt des SX1280 bedeutet (×46 auf 2,4 GHz). Der Teiler 12 ist
gerade, also echter Integer-Modus.

**Nicht ungefragt ändern** — greift in den funktionierenden Sendetakt ein.

### 2.6 GPS bekommt keinen Fix

Langjähriges Problem, blockiert **jeden echten Sendebetrieb**. `sats=0`,
`vis=0..2`. `gpsdo_is_ready()` wird nie wahr, `tx_allowed()` bleibt falsch. Das
Gating arbeitet korrekt, es fehlt nur die Zeit.

Prüfschritte: siehe Abschnitt 8.

Behelf: `gpsgate 0` hebt die Sperre auf. Der NEO-7M liegt dann bei ±2,5 ppm, auf
2,4 GHz also bis ±6 kHz daneben. Für Tests am eigenen Empfänger brauchbar, für
QO-100 nicht.

---

## 3. Frequenzplan und seine Begründung

```
QO-100-Schmalbandtransponder  10489,500 … 10490,000 MHz (500 kHz)
untere CW-Bake                10489,500 MHz
Bullseye-LNB LO                9750,000 MHz  (Low-Band, KEIN 22-kHz-Ton)
                                             13 V — Transponder sendet vertikal
LNB-Ausgang                     739,500 …  740,000 MHz
Mischoszillator                 720,000 MHz  (5. Oberwelle von 144 MHz)
ZF                               19,500 …   20,000 MHz
Spiegel                         700,500 MHz  (5,3 % entfernt, filterbar)
```

Warum genau 19,5 MHz — drei Bedingungen, die sich überschneiden:

1. **Der Si4732 kann SSB nur im AM/SW-Zweig**, 2,3–26,1 MHz laut Datenblatt (die
   PU2CLR-Bibliothek nennt praktisch 150 kHz–30 MHz). Der LNB-Ausgang bei
   739,5 MHz liegt Faktor 28 darüber, ein Mischer ist Pflicht. Der FM-Bereich
   (64–108 MHz) hilft nicht: noch Faktor 7 daneben, und FM kann nur
   Breitband-FM mit 150 kHz Kanalbandbreite.
2. **Der Mischoszillator muss GPS-geführt und ganzzahlig geteilt sein.** Die
   5. Oberwelle hebt das Phasenrauschen um 20·log₁₀(5) = 14 dB mit an. 144 MHz
   ist die einzige Frequenz in der Nähe, die PLL B ganzzahlig erreicht:
   864 ÷ 6 = 144. (720 ÷ 144 = 5 ist kein zulässiger MultiSynth-Teiler.)
3. **Der Spiegel muss filterbar sein.** Bei 19,5 MHz liegt er 39 MHz = 5,3 %
   entfernt. Eine früher erwogene ZF von 10,7 MHz hätte ihn auf 2,9 % gelegt —
   nicht filterbar.

**Verworfene Alternative:** LNB-Oszillator über seine Referenz verschieben
(26,845 MHz statt 25 MHz → LO 10469,5 MHz → Signal direkt bei 20 MHz, Mischer
gespart). Scheitert dreifach: (1) die Ausgangsstufe des LNB ist für
950–2150 MHz gebaut und das Bias-Tee muss einen Hochpass gegen 13/18 V und den
22-kHz-Ton haben — 20 MHz kommen aus der F-Buchse nicht heraus; (2) der Spiegel
läge 0,4 % entfernt; (3) 26,845 MHz sind nur fraktional erzeugbar und werden
×390 multipliziert.

---

## 4. Taktverteilung

**Vier Signale werden gebraucht, zwei SI5351 sind vorhanden.** Ein einzelner
SI5351 kann es nicht: er hat zwei PLLs, und aus 24 MHz Referenz braucht jede der
drei Frequenzen eine eigene PLL-Frequenz, wenn die Teiler ganzzahlig bleiben
sollen (25 MHz → nur PLL 600; 52 MHz → nur PLL 624; 144 MHz → nur PLL 864).

### GPSDO 1 — SI5351 am Pico, I2C0 (GP0/GP1), Code in `gpsdo.c`

Referenz: 24 MHz von NEO-7M TIMEPULSE, UART1 auf GP4/GP5.

| | |
|---|---|
| PLL A | 884 MHz = 24 × (36 + 5/6), **fraktional**, p1=4202, p2=4, p3=6 |
| CLK1 | ÷17 = **52 MHz**, 4 mA → SX1280 XTA. Registers MS1 = 50–57, p1=1664 |
| PLL B | 864 MHz = 24 × 36, ganzzahlig, p1=4096, p2=0, p3=1. Regs 34–41 |
| CLK2 | ÷6 = **144 MHz**, 8 mA → Mischer-LO. Regs MS2 = 58–65, p1=256 |
| CLK0 | abgeschaltet (0x80) |

Steuerregister: `CLK1_CTRL = 0x4D`, `CLK2_CTRL = 0x6F`, `CLK0_CTRL = 0x80`.

Output-Enable ist **aktiv LOW**, Bit 0 = CLK0, Bit 1 = CLK1, Bit 2 = CLK2:
`0xFD` = nur CLK1 (Bootzustand), `0xF9` = CLK1 + CLK2.

**8 mA auf CLK2 ist Absicht** — das Nutzsignal ist die 5. Oberwelle und lebt von
der Flankensteilheit.

**PLL-Reset** (`0xA0`, Register 177) nur beim Init, dort ist es sicher, weil der
SX1280 noch im Reset hängt. **Zur Laufzeit darf PLL A nie zurückgesetzt werden**,
der SX1280 verliert sonst für Millisekunden seinen Takt. Nur PLL B: `0x80`.

### GPSDO 2 — SI5351 am Arduino Pro Micro, eigener Sketch

War für die 25-MHz-Referenz des LNB gedacht — **wird dafür nicht gebraucht**,
weil das Bullseye keine Referenz annimmt. Liefert jetzt die 25 MHz für den
Si4732-RCLK. Im Sketch steckt ein Schalter `CLK1_TEST_144MHZ`, der CLK1 zwischen
25 MHz und 144 MHz umstellt (Messbetrieb).

**Achtung, Verwechslungsgefahr:** Beide SI5351 benutzen CLK2 — der im Pico für
die 144 MHz, der im zweiten GPSDO für die 25 MHz.

### Der LNB hängt NICHT am GPS

Das Othernet-Bullseye nimmt keine externe Referenz an; die 25 MHz an seinem roten
Anschluss sind ein **Ausgang**, laut Hersteller fest. TCXO mit ±2 ppm, LO-Stabilität
±10 kHz bei 23 °C und ±30 kHz von −20 bis +60 °C. LO werkseitig 9750 / 10600 MHz.

Damit ist der LNB die dominierende Fehlerquelle, und sein Fehler geht 1:1 in die
ZF. Kompensation über `rx cal <Hz>` (Abgleich an der Bake, siehe Abschnitt 6).

Spätere Lösung (nicht gebaut, bewusst verschoben): Die 25 MHz des Bullseye gegen
GPS zählen. Der LO ist exakt 390 × Referenz, der Fehler also exakt
390 × Referenzfehler — laufend und inklusive Temperaturdrift. Dass der Ausgang im
warmen Gehäuse mitdriftet, ist dabei genau der Punkt: er driftet mit demselben
Oszillator wie der LO.

Dafür nötig: 1 PPS vom NEO-7M, PIO-Zähler, Korrekturrechnung. Genauigkeit
1 s → 390 Hz, 10 s → 39 Hz, 100 s → 4 Hz am LO.

**ACHTUNG:** `TIMEPULSE1` ist belegt. Der Kommentar in `gpsdo.c` behauptet
„TIMEPULSE2 = 24 MHz", das erste Nutzbyte des CFG-TP5-Pakets ist aber
`tpIdx = 0x00`, also TIMEPULSE1. Der freie Ausgang ist TIMEPULSE**2**, ein
zweites CFG-TP5 mit `tpIdx = 0x01`.

---

## 5. Hardware, Verdrahtung, Pegel

Vollständig in `WIRING.txt`. Hier das Wesentliche.

### GPIO-Belegung (nichts Bestehendes wurde verschoben)

```
GPIO  0  SI5351 + Si4732 SDA     GPIO 14  RX_EN
GPIO  1  SI5351 + Si4732 SCL     GPIO 15  TX_EN
GPIO  2  Encoder A               GPIO 16  SPI0 MISO
GPIO  3  Encoder B               GPIO 17  SPI0 NSS
GPIO  4  GPS TX (UART1)          GPIO 18  SPI0 SCK
GPIO  5  GPS RX (UART1)          GPIO 19  SPI0 MOSI
GPIO  6  OLED SDA (I2C1)         GPIO 20  SX1280 NRESET
GPIO  7  OLED SCL (I2C1)         GPIO 21  SX1280 BUSY
GPIO  8  frei                    GPIO 22  TCXO_EN (unbenutzt)
GPIO  9  CW dit                  GPIO 25  LED
GPIO 10  Encoder Taster          GPIO 26  Mikrofon MAX4466 (ADC0)
GPIO 11  CW dah                  GPIO 27  Si4732 Audio (ADC1)
GPIO 12  Sidetone PWM            GPIO 28  frei (ADC2)
GPIO 13  Si4732 RST
```

GP8 und GP28 sind frei. **Beide sind verplant:** GP8 für die
Mikrofonbuchsen-Erkennung, einer von beiden für das 1 PPS der LNB-Disziplinierung.
Nicht doppelt belegen.

### Si4732-A10, 16-Pin SOP — nackter Chip auf Adapterplatine

```
 1  LOUT/DFS        16  ROUT/DOUT
 2  GPO3/DCLK       15  GND
 3  GPO2/INTB       14  VDD
 4  GPO1            13  RCLK
 5  NC              12  SDIO
 6  FMI             11  SCLK
 7  RFGND           10  SENB
 8  AMI              9  RST
```

**Drei Dinge, die ein Modul verbirgt und ein nackter Chip nicht:**

1. **Busmodus wird aus GPO1/GPO2 bei der STEIGENDEN Flanke von RST übernommen.**
   Offen lassen ist richtig: interner Pull-up auf GPO1 und Pull-down auf GPO2
   wählen 2-Draht. Aber RST muss mindestens 100 µs low bleiben, und **SCLK muss
   über die Flanke high sein**. Weil der SI5351 den Bus teilt, nimmt
   `si_hw_reset()` GP1 dem I2C-Block weg und treibt ihn von Hand high — sonst
   kommt der Chip im SPI-Modus hoch und antwortet nie. Zusätzlich: keine
   I2C-Start-Bedingung in den 300 ns vor der Flanke.
2. **RCLK ist Pin 13, ein eigener Pin.** Beim 16-Pin-Gehäuse gibt es kein
   XTAL1/XTAL2-Paar; im Quarzbetrieb läge der 32,768-kHz-Quarz zwischen Pin 13
   und Pin 2 (GPO3). Wir benutzen keinen Quarz: `XOSCEN = 0` im POWER_UP.
3. **SENB-Polarität ist zwischen Si4732-A10 und Si4735-D60 widersprüchlich
   dokumentiert.** Der Treiber tastet daher beide Adressen 0x11 und 0x63 ab und
   meldet, welche geantwortet hat.

Verdrahtung:

```
12 SDIO  → GP0 (gemeinsam mit SI5351)
11 SCLK  → GP1 (gemeinsam mit SI5351)
 9 RST   → GP13
13 RCLK  ← 25 MHz von GPSDO 2, über 1 kΩ / 220 Ω Teiler
 8 AMI   ← ZF über 10 nF (NICHT FMI — SSB läuft über den AM-Zweig)
 1 LOUT  → GP27 über Vorspannungsnetzwerk (10 µF, 2× 10 kΩ, 1 kΩ, 47 nF)
14 VDD   ← 3V3 (Pin 36), 100 nF nah am Chip + 10 µF
 7 RFGND → GND
 4 GPO1, 3 GPO2, 5 NC, 6 FMI, 2 GPO3, 16 ROUT → offen
10 SENB  → beliebig, Treiber tastet beide Adressen ab
```

RCLK-Teiler nötig, weil der SI5351 bei 8 mA etwa 2,4 Vpp liefert und der
RCLK-Eingang nur etwa 0,3–1,5 V verträgt. `REFCLK_PRESCALE = 800`,
`REFCLK_FREQ = 31250` (25 MHz / 800, exakt).

### RMS-11X-Mischer

```
Pin 1 = LO, Pin 4 = IF, Pin 5 = RF, Pins 2/3/6 = GND
RF/LO 5–1900 MHz, IF 5–1000 MHz, Mischverlust typ. 7,1 dB,
RF-Kompression +1 dBm, max. RF 200 mW, max. IF-Gleichstrom 40 mA
```

Ohmmeter-Test vor dem Löten: Pins 2/3/6 untereinander 0 Ω; Pins 1 und 5 gegen
Masse wenige Ω (Übertragerwicklungen); Pin 4 gegen Masse deutlich hochohmiger
(Diodenring). Das identifiziert IF eindeutig; LO gegen RF braucht die
Pin-1-Markierung.

**Keine Gleichspannung auf Pin 4.** Bias-Tee eines angeschlossenen SDR
ausschalten.

### Hochpass im LO-Zweig (gebaut und bewiesen)

```
CLK2 ──[5 pF]──┬──[5 pF]── SPF5189Z
               │
          13 mm Draht (≈ 10 nH)
               │
              GND
```

5 pF = 2× 10 pF SMD in Serie. Gemessen 34 dB bei 144 MHz, 0 dB bei 720 MHz.

Der Hochpass ist unkritisch: gebraucht werden nur etwa 12 dB, damit der
Verstärker nicht komprimiert. Brauchbar ist alles von 3,3 bis 12 pF mit 6,8 bis
15 nH. **Draht nicht unter 11 mm kürzen** — darunter gewinnt man bei 144 MHz
wenig und verliert bei 720 MHz messbar.

### Tiefpass am IF (NOCH NICHT GEBAUT — siehe 2.1)

### Pegelkette mit Istwerten

```
LNB-Ausgang, schwaches Signal   −60 dBm
SAW-Filter Band 12 DL           −2,5 dB
Mischer bei −5 dBm LO           −15 dB
Dämpfungsglied                  −3 dB
= am Si4732                     etwa −80 dBm
Empfindlichkeit Si4732 SSB      −110 dBm
Reserve                         etwa 30 dB
```

**Kein Vorverstärker im Empfangsweg.** Zwei unabhängige Gründe: (1) Das Rauschmaß
setzt der LNB allein (45–50 dB Verstärkung, 0,5–1 dB Rauschmaß); nach Friis
verschwindet alles Dahinterliegende — selbst 33 dB Rauschmaß der Folgekette
verschlechtern die Gesamtkette um unter 0,05 dB. (2) Der LNB liefert über seine
Bandbreite −20 bis −10 dBm Summenleistung; 20 dB Vorverstärkung brächten den
Mischer in die Kompression.

---

## 6. Firmwarestand

Branch `claude/qo100-rx-path`, Version **2.3.0-rx** (Sendeversion auf `master`
ist 2.2.0; die Versionsnummer unterscheidet sie, weil beide vorher 2.2.0 hießen
und das zu Verwechslungen geführt hat).

### Neue Dateien

| Datei | Inhalt |
|---|---|
| `si4732.c` / `.h` | Treiber, ~560 Zeilen |
| `tools/fetch_ssb_patch.sh` | holt den SSB-Patch |

### Geänderte Dateien

`main.c` (rx- und lo-Kommandos, Log-Senke, Persistenz, Init), `gpsdo.c`/`.h`
(PLL B + CLK2), `CMakeLists.txt`, `WIRING.txt`, `.gitignore`.

### Konsolenkommandos

```
lo [0|1]              144 MHz auf CLK2, beim Booten AUS
rx                    Statuszeile
rx diag               ausführlicher Bericht, inkl. gefundener I2C-Adresse
rx on | off           hoch- / herunterfahren (Hochfahren lädt den Patch)
rx usb | lsb | am     Demodulationsart
rx freq <kHz>         2300..26100, Dezimalen erlaubt (19500.5 = 19 500 500 Hz)
rx cal <Hz>           LNB-Fehler, wird im Flash gespeichert
rx vol <0..63>
rx bw <Hz>            nächstliegendes Filter wird gewählt
rx bfo <Hz>           roher BFO, wird vom nächsten Abstimmen überschrieben
rx mute 0|1
```

Statuszeile: `!R on= mode= hz= cal= bw= bfo= vol= mute= rssi= snr=`

### Der SSB-Patch

Der Si4732 kann SSB nicht von sich aus — **15 832 Byte** müssen bei *jedem*
Einschalten per I2C hineingeladen werden (1979 Achterblöcke, jeder beginnt mit
Opcode 0x15 oder 0x16; strukturell geprüft). Der Blob ist ein
Silicon-Labs-Patch, veröffentlicht von Vadim Afonkin, getragen von der
PU2CLR-SI4735-Bibliothek, deren Kopf kommerzielle Nutzung ausschließt.

Das Repo ist öffentlich und GPL, deshalb bleibt der Patch draußen:
`tools/fetch_ssb_patch.sh` holt `src/patch_full.h` von `pu2clr/SI4735`, entfernt
das AVR-`PROGMEM`, gibt dem Array internal linkage und hängt `SSB_PATCH_SIZE`
per `sizeof` an. `__has_include` entscheidet, ob die Seitenbandmodi existieren.
Ohne Patch baut alles und empfängt AM; CMake sagt es beim Konfigurieren, und
`rx usb` verweigert mit Erklärung.

Das Skript macht `touch si4732.c`, weil der Compiler nie eine Abhängigkeit auf
eine Datei aufzeichnet, die beim letzten Bau nicht existierte — sonst bekommt
man still eine AM-Firmware. **Das ist mir beim Testen selbst passiert.**

Upload-Dauer etwa 2–3 s bei 100 kHz Busfrequenz. Deshalb fährt der Empfänger
beim Booten nicht hoch, sondern erst mit `rx on`.

### Abstimmung und Kalibrierung

Der Chip nimmt nur ganze Kilohertz, was für SSB unbrauchbar ist. `si4732_tune_hz()`
rundet auf ganze kHz für den Tuner und gibt den Rest an den BFO (8-Hz-Schritte).
Der Kalibrier-Offset wird an dieser einen Stelle addiert, damit alles darüber in
echten Frequenzen arbeitet. Randfälle durchgerechnet: der Rest bleibt in ±500 Hz,
weit im BFO-Bereich.

`rx cal` ist definiert als **um wieviel alles zu hoch ankommt**:

```
Bake          10489,500 MHz
LNB-LO       − 9750,000 MHz
Mischer-LO   −  720,000 MHz   (GPS-exakt, trägt keinen Fehler bei)
= Soll-ZF        19,500 MHz
```

Bake bei 19,512 MHz gesehen → `rx cal 12000`.

**Die ZF ist der genaueste Messpunkt der Kette**: der ppm-Fehler des Messgeräts
wird nur mit 19,5 MHz multipliziert (2 ppm = 39 Hz), bei 739 MHz wären es 1,5 kHz.

### Persistenz

Letzter 4-KB-Flash-Sektor, Magic `0x51303130`, CRC32, `flash_safe_execute()`.
**Layout-Version 5** (von 4 erhöht für den Empfangsblock: `rx_cal_hz`, `rx_hz`,
`rx_mode`, `rx_vol`, `rx_bw_hz`). 6 Reservebytes sind noch frei.

`persist_apply()` läuft **vor** `si4732_init()`, deshalb gibt es
`si4732_restore()`, das Werte nur merkt, ohne die Hardware anzufassen. Das erste
`rx on` wendet alles an.

**Flashen löscht den Konfigurationssektor.** Das ist dokumentiertes Verhalten,
kein Fehler.

### Log-Senke statt printf

**`pico_enable_stdio_uart` und `pico_enable_stdio_usb` stehen beide auf 0.** Alle
`printf`-Aufrufe landen im Nichts — betrifft sämtliche `[GPSDO]`- und
`[SX1280]`-Diagnosen, die damit nie sichtbar waren. Der Si4732-Treiber benutzt
deshalb `si4732_set_log()`, das `main.c` auf die Konsole legt. `gpsdo.c` ist noch
nicht umgestellt (offener Punkt).

stdio **nicht** einschalten: `pico_enable_stdio_usb` kollidiert mit der eigenen
TinyUSB-Konfiguration (CDC + Audio).

---

## 7. Messprotokoll

| Datum | Messung | Wert |
|---|---|---|
| 04.10. | GPSDO 2, Grundwelle 144 MHz | +2,0 … +2,5 dBm |
| 04.10. | GPSDO 2, 5. Oberwelle 720 MHz | −16,5 … −17,5 dBm |
| 02.11. | Hochpass, 144 MHz nach Filter | −34 dBm (≈ 34 dB) |
| 02.11. | Hochpass, 720 MHz nach Filter | −19 dBm (≈ 0 dB) |
| 03.11. | SPF5189Z-Ausgang, 720 MHz | −9 dBm → später −5 dBm |
| 03.11. | Mischverlust (Generator −31 dBm) | ≈ 20 dB |
| 03.11. | LNB bei 739,5 MHz | Transponder sichtbar |
| 03.11. | Testsignal am IF hörbar bis | −35 dBm Generatorpegel, **nur mit AGC aus** |

### Messfallen, die uns Zeit gekostet haben

- **TinySA: LOW-Eingang nur bis 350 MHz, HIGH ab 240 MHz.** Der interne
  Abschwächer liegt **nur im LOW-Pfad**. 144 und 720 MHz werden also an zwei
  verschiedenen Empfängern mit eigener Kalibrierung gemessen — **absolute Pegel
  zwischen den beiden Frequenzen sind nicht vergleichbar.** Nur gleiche Frequenz
  gegen gleiche Frequenz vergleichen.
- **Oberwellen-Messung lügt bei Übersteuerung.** Der Analysator erzeugt die
  720 MHz selbst aus den 144 MHz. Test: Dämpfung um 10 dB erhöhen, der 720-MHz-Pegel
  muss um genau 10 dB fallen. Fällt er um 20–30 dB, kam er aus dem Mischer des
  Messgeräts.
- **TinySA Ultra: maximal +6 dBm bei 0 dB Dämpfung**, 1-dB-Kompression bei
  −1 dBm, Hersteller empfiehlt unter −25 dBm. Original-TinySA +10 dBm am LOW-Eingang.
- **RTL-SDR v3 im Direct Sampling** ist für das IF das falsche Gerät: Tuner
  überbrückt, kein Vorverstärker, 20–30 dB unempfindlicher. Außerdem läuft der ADC
  mit 28,8 MHz, Nyquist bei 14,4 MHz — 19,5 MHz liegen in der zweiten Nyquist-Zone
  und können auf 28,8 − 19,5 = 9,3 MHz herunterfalten, gespiegelt. **Ungeklärt, ob
  SDRangel das auflöst.** Nebenbei: 144 MHz = 5 × 28,8 MHz, alle Harmonischen des
  LO falten also auf exakt 0 Hz.
- **Für den LNB-Test den RTL-SDR im Normalmodus bei 739,5 MHz benutzen**, mit
  **20 dB Dämpfung davor** (der LNB liefert −20 bis −10 dBm Summenleistung, der
  Stick komprimiert ab etwa −30 dBm) und **Bias-Tee aus** (4,5 V gegen die 13 V
  aus dem Triplexer).

---

## 8. Prüfschritte GPS-Fix

In dieser Reihenfolge:

1. **Antenne:** freie Sicht zum Himmel. Erster Cold Start im Freien kann
   15 Minuten dauern (Almanach).
2. **Antennenspeisung:** aktive Patchantenne braucht Gleichspannung über das
   Kabel. Liefert das Modul sie? U.FL-Stecker richtig?
3. **HF-Störung durch das eigene Gerät:** Pico, SPI zum SX1280 und die
   52-MHz-Leitung strahlen; Oberwellen können ins L1-Band bei 1575,42 MHz fallen.
   Gegentest: GPS-Modul mit langem Kabel weit weg betreiben.
4. **Modul einzeln** an USB-Seriell-Adapter und u-center. Zeigt es dort auch
   nichts, ist es Antenne oder Modul.
5. **TIMEPULSE abklemmen** und sehen, ob dann ein Fix kommt — 24 MHz direkt am
   Modul könnten den Empfänger selbst belasten. Nur Diagnose, der SX1280 läuft
   dann ohne Takt.

---

## 9. FEHLER, DIE SCHON GEMACHT WURDEN

Damit sie sich nicht wiederholen.

1. **Messwerte gegen eine Referenz von vor einem Monat verglichen**, von einer
   anderen Quelle, mit anderen Kabeln und anderer Dämpfungseinstellung. Daraus
   folgte eine falsche Diagnose („es sind 1-pF-Kondensatoren verbaut"), die der
   Benutzer zu Recht zurückgewiesen hat. **Referenz immer in derselben Sitzung
   mit identischem Aufbau messen.**
2. **Das Filter hing an CLK2, der in der Firmware abgeschaltet war.** Gemessen
   wurde Übersprechen von CLK0 im Chip: −72 dBm bei 144 MHz und −45 dBm bei
   720 MHz, also 75 dB und 31 dB Isolation. Dass die Isolation mit der Frequenz
   schlechter wird, ist auch der Grund, warum die Grundwelle scheinbar unter ihrer
   eigenen 5. Oberwelle lag — was kein realer Oszillator tut. **Vor jeder Messung
   mit `lo 0` prüfen, dass das Signal verschwindet.**
3. **„Quarz entfernen, Takt auf XTAL1"** für den Si4732 empfohlen. Gilt für
   Module, nicht für das 16-Pin-Gehäuse — dort ist RCLK ein eigener Pin.
4. **Anfangs einen Tiefpass zur Spiegelunterdrückung empfohlen.** Falsch, dafür
   braucht es Bandpass oder Hochpass. Und eine ZF von 10,7 MHz hätte den Spiegel
   auf 2,9 % Abstand gelegt — unfilterbar.
5. **Angenommen, das Bullseye könne eine 25-MHz-Referenz annehmen.** Es kann
   nicht; der Anschluss ist ein Ausgang. Das hat den ganzen Disziplinierungsplan
   für die Empfangsseite gekippt.
6. **CLK2 beim Booten eingeschaltet gelassen.** `gpsdo_init()` läuft vor dem
   Boot-Gate, damit konnte ein Fehler im Empfangszweig den Sender am Starten
   hindern. Behoben.
7. **Die fehlenden 9 dB am Verstärker als „die eigentliche Baustelle"
   bezeichnet.** In Signalpegeln richtig, in Empfindlichkeit nicht: 0,3 dB
   Systemrauschmaß. Bei einem LNB mit 45–50 dB Verstärkung ist fast alles
   dahinter egal.
8. **Den Tiefpass am IF komplett übersehen** — die Ursache des aktuellen
   Blockers. Hochpass im LO-Zweig und Bandpass im HF-Zweig waren geplant, am IF
   stand nur ein Dämpfungsglied.
9. **Nicht geprüft, dass 144- und 720-MHz-Messungen von verschiedenen
   TinySA-Eingängen kamen** und damit nicht vergleichbar sind.
10. **Nach dem Bestand gefragt, nachdem konkrete Bauteilwerte empfohlen waren**,
    statt vorher. Der Benutzer hatte keine passenden Spulen — die Drahtlösung
    kam erst danach.

---

## 10. UNGEPRÜFTE ANNAHMEN

| Annahme | Status | Wie zu prüfen |
|---|---|---|
| **BFO-Vorzeichen** in `si4732_tune_hz()`: positiv schiebt nach oben | **angenommen** | Auf die Bake abstimmen, `rx bfo 500`, hören ob der Ton steigt. Fällt er, eine Negation umkehren — mehr nicht. Im Code kommentiert. |
| Si4732 RCLK-**Eingangspegel** 0,3–1,5 V | aus dem Gedächtnis von AN332 | Datenblatt. Das Fenster 31130–34406 Hz ist **belegt**. |
| SENB-Polarität A10 | widersprüchlich | Treiber tastet beide Adressen ab, `rx diag` meldet die gefundene |
| Mischverlust-Kennlinie über LO-Pegel | an eine Messung gefittet | mehr Messpunkte |
| Bakenpegel am LNB-Ausgang ≈ −63 dBm | geschätzt | **nie gemessen** — bei 739,5 MHz ablesen, dann −20,5 dB für die ZF |
| Spur bei 20,000 MHz (PLL A − PLL B) | Theorie | Trägerfrequenz feststellen, `lo 0` gegentesten |
| RMS-11X-Pinbelegung | aus indexiertem PDF | Ohmmeter-Test (Abschnitt 5) |
| Si4732 gesamter Treiber | **kompiliert, nie gegen Hardware getestet** | anschließen |

---

## 11. Nächste Schritte in Reihenfolge

1. **Tiefpass am IF bauen** (27 pF / 120 nH / 27 pF), Dämpfungsglied davor.
   Dann Bake bei 19,5 MHz suchen. **Das ist der aktuelle Blocker.**
2. Falls der Träger bleibt: seine Frequenz feststellen, `lo 0` gegentesten. Bei
   20,000 MHz PLL A auf 624 MHz umstellen (Abschnitt 2.5).
3. **Bakenpegel bei 739,5 MHz messen**, um die Schätzung zu ersetzen.
4. **Si4732 anschließen**, `rx diag` / `rx on` / `rx usb` / `rx freq 19500`.
   Ton zunächst an LOUT Pin 1 über 10 µF, Line-Pegel — nicht die
   Gerätekopfhörerbuchse, die hängt am PWM-Sidetone.
5. **`rx cal`** aus der Istfrequenz der Bake setzen.
6. **BFO-Vorzeichen prüfen** (Abschnitt 10).
7. Ursache von „waiting for clock" einkreisen (Abschnitt 2.4).
8. **Stufe R2:** Audio über ADC1 in den Pico, Kopfhörermischer aus Sidetone,
   Mikrofon-Mithören und RX, mit Mute-Regeln. Mikrofon und RX nie gleichzeitig,
   deshalb umschalten statt Round-Robin.

---

## 12. Bauen

```bash
git fetch origin
git checkout claude/qo100-rx-path && git pull
git submodule update --init --depth 1 external/tinyusb   # fehlt leicht
sh tools/fetch_ssb_patch.sh                              # einmalig
export PICO_SDK_PATH=/pfad/zum/pico-sdk                  # SDK 2.1.1
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

Beim Konfigurieren muss `Si4732: SSB patch found` erscheinen. Steht dort
`missing`, wird der Empfänger nur AM.

Werkzeuge: `arm-none-eabi-gcc`, `cmake`, Pico SDK **2.1.1**, Board `pico2`
(RP2350). Flashen: BOOTSEL halten, `build/SX1280SDR.uf2` auf `RPI-RP2` kopieren.
`version` in der Konsole muss **2.3.0-rx** melden.

---

## 13. Vorhandene und fehlende Teile

**Vorhanden:** Bullseye-LNB, Triplexer (DC + HF + vorbereitet für GPS auf der
Leitung), RMS-11X, SPF5189Z, SAW-Filter Band 12 DL, TinySA, RTL-SDR v3, zwei
GPSDO, Spiegel. Bauteile: SMD-Kondensatoren 1, 10, 15, 27 pF; Spulen 120 nH und
2,2 µH (für HF-Filter unbrauchbar, Drahtstücke verwenden: 13 mm 0,5-mm-Draht
≈ 10 nH).

**Fehlt:** Si4732-A10-Modul ist vorhanden, aber **noch nicht angeschlossen**.
NanoVNA wäre nützlich (TinySA kann keine Übertragungsfunktion messen).

---

## 14. Arbeitsweise, die sich bewährt hat

- **Stufenweise bauen und nach jeder Stufe messen.** Der Benutzer hat das
  ausdrücklich eingefordert und es hat jeden Fehler lokalisiert.
- **Registerwerte unabhängig zurückrechnen, bevor sie festgeschrieben werden.**
  Hat beim SI5351 funktioniert.
- **Datenblätter nachsehen statt aus dem Gedächtnis antworten.** Jedes Mal, wenn
  das unterlassen wurde, kam ein Fehler aus Abschnitt 9 heraus.
- **Den Sendebetrieb nicht gefährden.** CLK1 (52 MHz) ist der einzige Takt des
  SX1280. Nie abschalten, PLL A nie zur Laufzeit zurücksetzen.
- Der Benutzer ist Funkamateur (DL1OKE) und baut selbst. Begründungen und
  Rechnungen sind erwünscht, Bevormundung nicht.
