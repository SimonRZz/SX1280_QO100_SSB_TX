# Minimalaufbau zur Störsuche (Bench-Firmware)

Ziel: Pico 2 und Si4732 allein, dann Teil für Teil ergänzen und nach jedem
Schritt hören bzw. das Spektrum ansehen. Kein Kopfhörer, das Audio geht über
USB an den PC (GUI-Reiter **Bench / Audio**).

## Firmware

    cmake -S . -B build-bench -DBENCH_MINIMAL=ON -DPICO_BOARD=pico2
    cmake --build build-bench

Gegenüber der normalen Firmware: keine Wartestufe für die SI5351-PLL, kein
SX1280, kein Core 1, kein GPS-Dienst, keine OLED-Aktualisierung, keine
automatische Speicherung. Si4732 mit **XOSCEN** (eigener 32,768-kHz-Quarz),
`rx cal 0`, Kopfhörer-PWM GP12 **aus** (`hpout 0`, Pin fest auf low).

## Verdrahtung Stufe 1–2

    Si4732 14 VDD   <- 3V3, 100 nF direkt am IC
    Si4732 15 GND   -> GND
    Si4732 12 SDIO  <-> GP0  (4,7 kOhm nach 3V3)
    Si4732 11 SCLK  <-  GP1  (4,7 kOhm nach 3V3)
    Si4732  9 RST   <-  GP13
    Si4732 10 SENB  -> Adresse wie bisher gebrückt
    Si4732 13 RCLK  und  2 GPO3: 32,768-kHz-Quarz dazwischen,
                    je Pin ein Lastkondensator nach GND (Wert nach
                    Lastkapazität des Quarzes, typ. 22 pF; Datenblatt prüfen)
                    SI5351 CLK2 NICHT an Pin 13!
    Si4732  8 AMI   <- Drahtantenne (ein paar Meter) über 100 nF
    Si4732  1 LOUT  -> 10 uF -> 10k/10k-Teiler an 3V3 -> 1 kOhm/47 nF -> GP27

Versorgung: Laptop im Akkubetrieb oder Powerbank.

## Befehle (alle auch in der GUI)

    rx stream 0|1|2     Audio an den PC: 1 roh (ADC, ohne AGC), 2 verarbeitet
    rx mon thru 0|1     ADC hörbar, auch wenn der Si4732 aus ist
    hpout 0|1           Kopfhörer-PWM GP12 an/aus
    smps pwm 0|1        Pico-Schaltregler im festen PWM-Modus
    rx lsb | usb | am, rx off, rx freq <kHz>, rx vol 0..63, rx bw <Hz>

Datenformat: Zwischen den Textzeilen stehen Binärpakete `FF A5 n` plus
n x int16 little-endian bei 8000 Hz. 0xFF kommt in UTF-8-Text nicht vor.

## Ablauf

1. Si4732 aus, `rx mon thru 1`, `rx stream 1`: Eigenrauschen des
   Pico-Audiowegs. Spektrum ansehen, `smps pwm 0/1` und `hpout 0/1`
   vergleichen. Testton vom Handy über 10 uF an GP27.
2. Si4732 mit Quarz und Draht, `rx lsb`, `rx freq 7100` (abends) oder `rx am`
   auf einem KW-Rundfunksender.
3. Si4732 aus eigenem 3,3-V-LDO.
4. Takt vom SI5351 statt vom Quarz (normale Firmware oder `rx xosc 0`).
5. OLED, GPS, SX1280, Mikrofonverstärker einzeln dazu.
6. Mischer und LO, zuletzt das LNB.

Nach jedem Schritt in der GUI eine WAV-Aufnahme machen (Knopf
„Aufnahme“). Die Linien im Spektrum sind mit ihrer Frequenz beschriftet:
50/100 Hz Netz, feste Töne aus Schaltreglern oder PWM, breites Rauschen.
