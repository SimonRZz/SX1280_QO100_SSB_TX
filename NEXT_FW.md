# Vorgemerkt für die nächste Firmware-Version

## Beschlossen

### LNB-Ablage als Vorgabe eintragen

Gemessen: Bullseye-LNB-LO bei **9750,0043 MHz**, also 4,3 kHz zu hoch.
Downlink-Signale kommen damit 4,3 kHz **zu tief** in der ZF an.

Nach der Konvention des Treibers (`si4732.h`: Wert = wie viel ein Signal zu
**hoch** ankommt) ist das **`rx cal -4300`**. Vorgabe von `s_cal_hz` in
`si4732.c` entsprechend setzen.

Zu beachten:

* `rx_cal_hz` wird bereits im Flash gespeichert. Ein Gerät, das schon einen
  Wert hat, behält ihn; die neue Vorgabe greift nur bei leerem oder
  zurückgesetztem Speicher. Bis zum nächsten Build reicht von Hand
  `rx cal -4300`.
* Die 4,3 kHz wurden mit dem RTL-SDR gemessen, dessen eigener Fehler steckt
  mit drin. Solange der TinySA den Mischer-LO macht, kommt dessen Fehler
  (bis ±7 kHz bei 720 MHz) noch dazu. Sauber stimmt der Wert erst mit dem
  GPS-gebundenen eigenen LO.
* Der Wert driftet mit der Temperatur des LNB-TCXO (±2 ppm, bis ±30 kHz über
  den vollen Bereich). Langfristige Lösung: Aufgabe #4, LNB gegen GPS
  disziplinieren.

## Vorgeschlagen, noch nicht beschlossen

* 32,768 kHz für RCLK aus PLL A statt aus PLL B, damit CLK0 (Mischer-LO,
  144 MHz) gleichzeitig laufen kann. PLL A bleibt dabei unverändert auf
  884 MHz: 884 MHz / (210 + 12485/16384) / 128 = 32768 Hz exakt.
* BFO 500 Hz als Vorgabe für SSB (damit war der Ton am klarsten).
* Korrekte Bakenfrequenzen in Hilfe und Doku (LO 720,137 MHz):
  untere CW 19,3587 MHz, BPSK 19,6087 MHz, obere CW 19,8587 MHz.
* Empfangsseite zeigt Downlink-Frequenz (10489,xxx MHz) statt ZF, aus
  LNB-LO, Mischer-LO und Kalibrierung errechnet.
