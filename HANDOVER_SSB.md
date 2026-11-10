# Übergabe: Si4732-A10 verriegelt nach jedem SSB-Tune

> **GELÖST am 2026-11-10.** Ursache: `REFCLK_FREQ` steht nach POWER_UP per
> Vorgabe auf 32768 und `REFCLK_PRESCALE` auf 1, der Treiber korrigierte das
> aber erst **nach** dem Patch-Upload. Der Chip verbrachte den gesamten Upload
> mit einer falschen Vorstellung seines eigenen Takts, und die gepatchte DSP
> hing dann beim ersten Tune. Abhilfe: `XOSCEN=1` nur beim Patch-POWER_UP, so
> wie es die Referenzbibliothek in `patchPowerUp()` macht. Commits `1b42d8d`
> und `7cbc24c`.
>
> Dieses Dokument bleibt als Protokoll der Fehlersuche stehen — die neun
> widerlegten Theorien in §5 und die Messfallen in §6 sind weiterhin gültig
> und ersparen Wiederholungen.
>
> **Das offene Problem steht jetzt in `HANDOVER_PEGEL.md`.**


**Stand:** 2026-11-09, 21:30 UTC+1
**Repo:** `SimonRZz/SX1280_QO100_SSB_TX`, Branch `claude/review-upstream-changes-uky0w`
**Letzter Commit:** `861970a`
**Dateien:** `si4732.c`, `si4732.h`, `main.c`, `gpsdo.c`, `WIRING.txt`

---

## 1. Das Problem in einem Satz

Der Si4732-A10 arbeitet in **AM einwandfrei**. Sobald in **SSB** (Patch geladen) das
Tune-Kommando abgesetzt wurde, beantwortet der Chip **kein einziges Kommando mehr**, das
eine Antwort zurückliefert — CTS bleibt dauerhaft auf 0, bis zum nächsten POWER_UP.

---

## 2. Fehlersignatur, exakt

Aus `rx ssbtest` (schrittweiser Hochlauf, Statusbyte nach jedem Kommando):

```
[SSBTEST] probe after reset: 1
[SSBTEST] POWER_UP(patch): 1
[SSBTEST] after POWER_UP       status=0x80  ok
[SI4732] SSB patch uploaded: 15832 bytes in 2900 ms
[SSBTEST] after patch          status=0x80  ok
[SI4732] part 32, fw 1.0, chiprev 0x41, patch id 0x9D29
[SSBTEST] after GET_REV        status=0x80  ok
[SSBTEST] after PRESCALE       status=0x80  ok
[SSBTEST] after REFCLK_FREQ    status=0x80  ok
[SSBTEST] after SOFT_MUTE      status=0x80  ok
[SSBTEST] after SSB_MODE       status=0x80  ok
[SSBTEST] after VOLUME         status=0x80  ok
[SSBTEST] after BFO            status=0x80  ok
[SSBTEST] SSB_TUNE_FREQ (FAST) written: 6 of 6 bytes
[SSBTEST] 10 ms after tune     status=0x80  ok      <-- Tune akzeptiert
[SSBTEST] 100 ms after tune    status=0x80  ok
[SSBTEST] 500 ms after tune    status=0x80  ok      <-- noch gesund
[SSBTEST] GET_REV after tune: wrote 1 read 9 -> 00 00 00 00 00 00 00 00 00
[SSBTEST] after GET_REV        status=0x00  busy, CTS still low
[SSBTEST] RSQ 6 bytes: wrote 2 read 6 -> 00 00 00 00 00 00
[SSBTEST] after RSQ            status=0x00  busy, CTS still low
[SSBTEST] TUNE_STATUS: 0  -> 0 kHz
[SSBTEST] after TUNE_STATUS    status=0x00  busy, CTS still low
```

**Die entscheidenden Punkte:**

* Alle Kommandos **vor** dem Tune: `status=0x80`, ERR-Bit nie gesetzt.
* Der Tune selbst wird angenommen (6 von 6 Bytes quittiert), CTS kommt zurück und bleibt
  bei 10, 100 und **500 ms** sauber auf 0x80.
* Das **erste** Kommando danach — egal welches — schlägt fehl. Auch `GET_REV` (0x10),
  das vor dem Tune jedes Mal funktioniert und das nicht „im Patch fehlen" kann.
* Der Chip **quittiert seine I2C-Adresse weiterhin** und liefert aktiv `0x00` zurück
  (nicht `0xFF` = offener Bus). Er ist also elektrisch am Bus, meldet aber permanent
  „busy".
* **Schreibzugriffe allein** (SET_PROPERTY) scheinen weiterhin quittiert zu werden, aber
  ihr internes CTS-Warten läuft dann ebenfalls in den Timeout.
* `rx am` (Hardware-Reset + POWER_UP) holt den Chip **jedes Mal sofort** zurück:
  `rssi=64`, `tune ok`. Kein Stromabziehen nötig.

---

## 3. Hardware

```
Pico 2 (RP2350) --- I2C0 (GP0 SDA / GP1 SCL) --- SI5351 + Si4732 (gemeinsamer Bus)
Si4732-A10, 16-pin SOP, Adresse 0x11
  Pin 1  LOUT  -> 10 uF -> Teiler 10k/10k an 3V3 -> 1k -> GP27 (ADC1), 47 nF gegen GND
  Pin 8  AMI   <- ZF vom Mischer (RMS-11X) ueber Tiefpass
  Pin 9  RST   <- GP13
  Pin 13 RCLK  <- SI5351 CLK2, 25 MHz (oder 36 MHz, umschaltbar), 8 mA
  Pin 14 VDD   <- 3V3(OUT) des Pico, Pin 36
  Pin 15 GND
  Pin 3/4 (GPO2/GPO1) offen -> 2-Wire-Bus-Modus beim RST-Anstieg gelatcht
  Pin 10 SENB  auf festem Pegel
KEIN Quarz vorhanden. XOSCEN=0, Takt ausschliesslich ueber RCLK.
```

**Wichtig:** `XOSCEN=1` ist auf dieser Hardware nicht benutzbar (kein Quarz). Ein Versuch
damit hat auch den AM-Empfang zerstoert, siehe Warnung in `main.c` bei `rx xosc`.

**Taktversorgung:** SI5351 CLK2. 25 MHz entsteht ueber einen **fraktionalen** MultiSynth
(864 MHz / (34+14/25)), 36 MHz ueber einen **ganzzahligen** (864/24). Beide getestet,
siehe unten.

---

## 4. Was nachweislich funktioniert

| | Nachweis |
|---|---|
| I2C, Reset, Bus-Modus-Latch | Adresse 0x11 antwortet, `probe after reset: 1` |
| Referenztakt | `rclk` meldet ON; ohne ihn stimmt **auch AM** nicht ab |
| SSB-Patch-Upload | 15832 Bytes, `patch id 0x9D29` wird korrekt zurueckgelesen |
| Chip-Identifikation | part 32, fw 1.0, component 6.0, chiprev 0x41 ('A') |
| **AM vollstaendig** | Abstimmung, RSSI, SNR, AGC-Status, Audio, Rundfunkempfang mit Musik |
| Si4732-Linearitaet | TinySA an Pin 8: 100 dBuV -> rssi 99; linear bis 31 dBuV, ±2 dB |
| Audiopfad | LOUT -> ADC1 -> PWM -> Kopfhoerer, mit Software-AGC (`rx mon`) |

Die AM-Seite ist also als Referenz voll nutzbar und zeigt, dass Hardware, Takt, Bus und
Treiberrahmen in Ordnung sind.

---

## 5. Widerlegte Theorien — bitte NICHT erneut probieren

Alle einzeln gemessen, alle ohne Wirkung auf den Fehler:

| # | Theorie | Wie widerlegt |
|---|---|---|
| 1 | FAST-Bit fehlt im Tune | Referenzbibliothek setzt `FAST=1` bei SSB immer. Eingebaut (`ARG1=0x81` USB / `0x41` LSB). **Keine Aenderung.** |
| 2 | Zu wenig Zeit nach dem Patch | 8 Versuche a 40 ms, zusaetzlich 100 ms Ruhe nach dem Upload. **Keine Aenderung.** |
| 3 | GET_INT_STATUS-Polling schaedigt | Polling entfernt. **Keine Aenderung.** |
| 4 | Falsche Transaktionsform (CTS-Poll vor Antwortlesen) | Antwort in einem Zugriff gelesen. **Keine Aenderung.** Ausserdem macht `waitToSend()` der Referenz genau dasselbe wie der eigene Code. |
| 5 | Falsche Antwortlaenge (8 statt 6 Byte) | Auf 6 umgestellt fuer SSB. **Keine Aenderung.** |
| 6 | `ANTCAP=0` loest endlose Varaktor-Suche aus | Referenz setzt `ANTCAPL=1` fest. Default auf 1 geaendert. **Keine Aenderung.** |
| 7 | Nur 0x42/0x43/0x47 sind im Patch nicht implementiert | **`GET_REV` (0x10) stirbt nach dem Tune ebenfalls.** Damit ist es kein Kommandoproblem. |
| 8 | Jitter/Nebenlinien des fraktionalen 25-MHz-Referenztakts | Auf 36 MHz aus ganzzahligem Teiler umgestellt (`rclk freq 36`). **Keine Aenderung.** |
| 9 | Zweites POWER_UP nach dem Patch fehlt | Komplette Referenzsequenz nachgebaut (siehe §7). **Keine Aenderung.** |

**Zusaetzlich widerlegt, aber als Diagnosefehler lehrreich:**

* `tuner reports 0 kHz` war **ueber Wochen** die Fehlbeschreibung. Ursache war teils ein
  abgeschalteter Referenztakt (CLK2 ist nach jedem Boot aus), teils ein **Altwert**:
  `si_read_tune_status()` liess `s_read_khz` bei einem fehlgeschlagenen Lesen unveraendert
  stehen, sodass ein AM-Ergebnis unter einem SSB-Tune erschien. Behoben in `c178d7c`.
* `last error: none` bei jedem CTS-Timeout: `si_wait_cts()` wollte die spezifischere
  Meldung `"no answer..."` erhalten und testete dafuer `s_last_err[0] == 'n'` — **`"none"`
  faengt ebenfalls mit n an.** Jeder echte Timeout meldete sich als „kein Fehler".
  Behoben in `24416b0`. **Alle Messungen vor diesem Commit sind in ihrer Fehlerbegruendung
  unbrauchbar.**

---

## 6. Messfallen, in die diese Sitzung gelaufen ist

Fuer die naechste KI, damit sie nicht dieselbe Zeit verliert:

1. **Der Chip bleibt nach einem Fehlversuch tot.** Jede weitere Messung befragt denselben
   verriegelten Zustand. **Vor jedem Experiment `rx am` ausfuehren** (Hardware-Reset +
   POWER_UP), sonst misst man nichts.
2. `rx usb` loest beim Beenden eine Statuszeile aus, die ihrerseits RSQ liest — und damit
   den Chip bereits verriegelt, bevor man selbst etwas messen kann. Deshalb existiert
   `rx ssbtest`.
3. CLK2 (Referenztakt) und CLK0 (Mischer-LO) sind **nach jedem Reset aus**. `rx am` /
   `rx usb` / `rx scan` schalten CLK2 inzwischen selbst ein, CLK0 nicht.
4. `rx scan` stimmt nach dem Durchlauf **nicht** auf den gefundenen Sender ab.
5. Der TinySA darf als LO **nicht auf 720.000 MHz** stehen: dort laeuft sein ADF4351 im
   Integer-N-Modus und erzeugt ueber Referenz-Nebenlinien ein Mischprodukt bei exakt
   20000 kHz an der ZF, das wie eine Bake aussieht (snr 16). Krumme Frequenz verwenden,
   z. B. 720.137 MHz.

---

## 7. Aktueller Code

### 7.1 Hochlaufsequenz (`si4732_set_mode`, `si4732.c`)

Folgt Schritt fuer Schritt der Referenzbibliothek pu2clr/SI4735:

```c
si_hw_reset();                              // RST low >=100 us, SCL hoch gehalten
si_probe(s_addr);
si_power_up(want_patch);                    // POWER_UP, PATCH=1, XOSCEN=0, FUNC=1, OPMODE=0x05
si_download_patch();                        // 15832 Bytes, 1979 Bloecke a 8 Byte
si_read_rev(want_patch);                    // nur hier ist patch_id lesbar
// ab hier s_on = true
if (want_patch) {
    si_apply_bandwidth();                   // SSB_MODE = 0x8010 | audiobw  (= setSSBConfig)
    si_power_up(false);                     // zweites POWER_UP, PATCH=0  (= radioPowerUp)
}
si_set_prop(PROP_REFCLK_FREQ,     rclk_hz / presc);
si_set_prop(PROP_REFCLK_PRESCALE, presc);
si_set_prop(PROP_AM_SOFT_MUTE_MAX_AT, 0);
si_apply_bandwidth();
si4732_set_volume(s_volume);
si4732_set_mute(s_mute);
if (want_patch) si4732_set_bfo_hz(s_bfo_hz);
si_tune();
```

### 7.2 Tune (`si_tune`)

```c
uint8_t arg1 = 0;
if (s_mode == SI4732_MODE_LSB)      arg1 = TUNE_SB_LSB | TUNE_FAST;   // 0x41
else if (s_mode == SI4732_MODE_USB) arg1 = TUNE_SB_USB | TUNE_FAST;   // 0x81
// AM: arg1 = 0x00 (kein FAST, Tune-Status wird dort gebraucht)

const uint8_t b[6] = { 0x40, arg1,
                       khz >> 8, khz & 0xFF,
                       s_antcap >> 8, s_antcap & 0xFF };   // s_antcap default 1
si_cmd(b, 6);

if (ssb) {                      // kein Ruecklesen in SSB
    sleep_ms(40);
    s_read_khz = SI_KHZ_NOT_READ;
    s_stc_ok   = si_wait_cts(200);
    return true;
}
// AM: si_wait_stc(300) + si_read_tune_status()
```

### 7.3 CTS-Warten und Kommando

```c
static bool si_wait_cts(uint32_t timeout_ms)   // 1-Byte-Lesezugriff, Bit 7 = CTS
{
    // 0xFF = offener Bus, gilt als "keine Antwort"
    // Meldet beim Timeout: kein ACK / Bus 0xFF / CTS blieb low
}

static bool si_cmd(const uint8_t *buf, size_t len)
{
    i2c_write_blocking(...);     // "I2C write failed" bei NAK
    return si_wait_cts(100);
}
```

### 7.4 Relevante Konstanten

```c
#define CMD_POWER_UP 0x01   #define CMD_GET_REV 0x10   #define CMD_POWER_DOWN 0x11
#define CMD_SET_PROPERTY 0x12   #define CMD_TUNE_FREQ 0x40   #define CMD_TUNE_STATUS 0x42
#define CMD_RSQ_STATUS 0x43   #define CMD_GET_INT_STATUS 0x14   #define CMD_AGC_STATUS 0x47
#define PROP_SSB_BFO 0x0100   #define PROP_SSB_MODE 0x0101
#define PROP_REFCLK_FREQ 0x0201   #define PROP_REFCLK_PRESCALE 0x0202
#define PU_PATCH 0x20   #define PU_XOSCEN 0x10   #define PU_FUNC_AM 0x01
#define PU_OPMODE_ANALOG 0x05
#define TUNE_FAST 0x01   #define TUNE_SB_LSB (1<<6)   #define TUNE_SB_USB (2<<6)
SSB_MODE = 0x8010 | audiobw   // SBCUTFLT=1, AVC aus, DSP_AFCDIS=1
```

---

## 8. Erkenntnisse aus der Referenzbibliothek

Quelle: `https://raw.githubusercontent.com/pu2clr/SI4735/master/src/SI4735.cpp`

* `setFrequency()` setzt im SSB-Zweig **`FAST = 1`**, trotz Kommentar „Used just on AM and FM".
* `setPowerUp()` setzt fuer AM/SSB fest **`ANTCAPH = 0`, `ANTCAPL = 1`** — nie 0.
* `patchPowerUp()` sendet ARG1 = `0b00110001` = **PATCH=1, XOSCEN=1**, FUNC=1.
  Kommentar: „This is a condition for loading the patch".
  → Auf dieser Hardware nicht nachvollziehbar (kein Quarz), Patch laedt trotzdem.
* `setSSB(usblsb)` ruft **`radioPowerUp()`** auf — ein **zweites POWER_UP mit PATCH=0**
  nach dem Upload. Danach werden REFCLK_FREQ und REFCLK_PRESCALE erneut gesendet.
* `waitToSend()` pollt CTS mit einem **separaten 1-Byte-Lesezugriff** vor dem Antwortlesen
  — identisch zum eigenen Code.
* `getCurrentReceivedSignalQuality()` liest fuer AM/SSB **6 Bytes** (`sizeResponse = 6`).
* `loadPatch()` ruft vorher **`queryLibraryId()`**: POWER_DOWN, 500 ms Pause, POWER_UP mit
  ARG1 = `0b00011111` (**FUNC=15** = Library-ID lesen, XOSCEN=1), 8 Bytes lesen,
  dann POWER_DOWN. **Dieser Schritt fehlt im eigenen Treiber noch.**

---

## 9. Offene Hypothesen, nach Prioritaet

### H1 — Spannungseinbruch an Pin 14 (aktuell wahrscheinlichste)

**Begruendung:** Der Ausloeser ist nicht ein Kommando, sondern der **Tune** — der Moment,
in dem die gepatchte DSP anfaengt zu rechnen und Strom zu ziehen. In AM wird dieser
Zustand nie erreicht, weil die ROM-Firmware deutlich weniger rechnet. Dass `rx am` den
Chip zuverlaessig zurueckholt, passt dazu: ein eingebrochener Chip braucht genau ein
POWER_UP.

Im Projekt steht seit Wochen die Notiz, die 3,3-V-Schiene sei grenzwertig. Der Si4732
haengt an `3V3(OUT)` des Pico, gemeinsam mit Display, SX1280 und SI5351.

**Noch nicht gemessen** (zweimal erbeten, noch nicht geliefert):
3V3 **direkt an Si4732 Pin 14** gegen Masse, einmal in AM und einmal direkt nach `rx usb`.
Unter ca. 2,9 V ist die Sache entschieden.

**Firmware-Test dafuer bereits eingebaut** (`rx ssbtest`, Commit `861970a`): Nach dem
Fehlschlag wird ein **blankes POWER_UP ohne Reset-Leitung** gesendet.

* Chip kommt zurueck → er war nie verriegelt, sondern weg. **H1 bestaetigt.**
* Chip bleibt tot → echte DSP-Verriegelung. **H1 widerlegt.**

**Abhilfe falls bestaetigt:** 100 uF + 100 nF unmittelbar an Pin 14/15; sauber: eigener
LDO nur fuer den Si4732 aus VBUS (vorhanden: MCP1702-3302, ua78m33, LM1117).

### H2 — `queryLibraryId()` fehlt

Der einzige verbliebene Unterschied zur Referenzsequenz. Die Bibliothek faehrt den Chip
vor dem Patch komplett herunter und startet ihn mit `FUNC=15`, bevor sie ihn mit dem
Patch-Bit startet. Billig nachzubauen:

```c
POWER_DOWN (0x11)
sleep_ms(500)
POWER_UP, ARG1 = 0x1F (FUNC=15, XOSCEN=1), ARG2 = 0x05
sleep_ms(3)
8 Bytes lesen
POWER_DOWN
```

Achtung: `XOSCEN=1` ohne Quarz — vermutlich mit `XOSCEN=0` zu versuchen.

### H3 — Patch passt nicht zu Si4732-A10

`patch_full.h` stammt aus pu2clr/SI4735 und ist der **Si4735-D60**-Patch. Der Chip meldet
`part=32` (Si4732), `chiprev=0x41` ('A'), `fw 1.0`, `component 6.0`. Die Patch-ID wird
zwar korrekt zurueckgelesen, was aber nur beweist, dass die Bytes angekommen sind. Die
pu2clr-Bibliothek nennt den Si4732-A10 ausdruecklich als unterstuetzt — trotzdem bisher
nicht unabhaengig geprueft.

---

## 10. Diagnosewerkzeuge in der Firmware

| Befehl | Zweck |
|---|---|
| `rx ssbtest` | Schrittweiser SSB-Hochlauf, Statusbyte nach jedem Kommando, danach GET_REV / RSQ / TUNE_STATUS einzeln, zum Schluss der Wiederherstellungstest |
| `rx raw <n> <hex...>` | Beliebiges Kommando senden, n Antwortbytes lesen, roh ausgeben, CTS-Zeit und ERR-Bit melden |
| `rx diag` | Takte, Chip-ID, Patch-ID, Modus, RSQ, Tune-Status, letzter Fehler |
| `rclk [0\|1]`, `rclk freq <25\|36>`, `rclk drive <2..8>` | Referenztakt |
| `lo [0\|1]` | Mischer-LO aus dem Geraet (CLK0, 144 MHz, 5. Oberwelle 720 MHz) |
| `rx mon [0\|1]`, `rx mon vol`, `rx mon agc` | Empfaengerton auf die Kopfhoererbuchse |
| `rx scan <von> <bis> <schritt>` | Pegeltabelle, bis 512 Punkte |
| `rx antcap <0..6143>` | Antennen-Abstimmkondensator, 0 = automatisch |
| `rx xosc <0\|1>` | POWER_UP-Quarzbit (auf dieser Hardware muss 0 bleiben) |

---

## 11. Unabhaengiges Zweitproblem (nicht verwechseln)

Fuer QO-100 fehlen **15 bis 20 dB** zwischen Mischerausgang und der Nachweisgrenze des
Si4732. Das ist ein reines Pegelproblem und hat mit dem SSB-Fehler nichts zu tun.

* Umgesetztes LNB-Rauschen am Si4732: 2 bis 14 dBuV (bw 4 kHz)
* Rauschgrenze des Si4732: ca. 15 dBuV (bw 4 kHz)
* Beweis, dass die HF-Kette arbeitet: Der RTL-SDR sieht am selben Mischerausgang
  QO-100-FT8 bei 19.436 MHz (LO 720.100). Sein Vorteil ist **nur** die
  Aufloesungsbandbreite (10 Hz FFT-Bin gegen 4 kHz Kanalfilter, rund 26 dB).

Loesung steht als Aufgabe #15 bereit: Emitterstufe mit S9018 oder 2N3904,
18k/4,7k Basisteiler an 5 V, 470 Ohm Kollektor, 33 Ohm unueberbrueckt + 47 Ohm
ueberbrueckt am Emitter, 47 Ohm Abschluss am Eingang. ~21 dB brutto, ~15 dB netto nach
dem Abschluss.

---

## 12. Frequenzplan zur Orientierung

```
QO-100 Bake unten   10489.500 MHz
QO-100 Bake mitte   10489.750 MHz  (400 Bd BPSK, durchgehend)
QO-100 Bake oben    10490.000 MHz
QO-100 FT8          10489.540 MHz
LNB-Oszillator       9750.004 MHz  (Bullseye, gemessen +4,3 kHz)
Mischer-LO            720.137 MHz  (TinySA HIGH output; NICHT 720.000!)
-> ZF Baken          19363 / 19613 / 19863 kHz
```

---

## 13. Was als Naechstes zu tun ist

1. `rx am` → `rx ssbtest` → **die Zeile `after bare POWER_UP` lesen.** Sie entscheidet
   zwischen H1 und H2/H3.
2. 3V3 an Si4732 **Pin 14** messen, in AM und direkt nach `rx usb`.
3. Je nach Ergebnis: Stuetzkondensatoren/eigener LDO (H1), oder `queryLibraryId()`
   nachruesten (H2).
4. Unabhaengig davon den ZF-Verstaerker bauen — ohne ihn ist QO-100 auch mit
   funktionierendem SSB nicht hoerbar.
