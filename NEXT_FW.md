# Firmwarestand und nächste Schritte

**Stand:** 2026-11-12. Diese Revision setzt die vier besprochenen Änderungen um.
Build/Softwareprüfungen und spätere Hardwaretests sind getrennt zu bewerten.
Die Ursache von Rauschen, Pulsieren und verzerrtem Ton ist weiterhin offen.

## In dieser Revision umgesetzt

1. **Kalibrierungsvorgabe −4300 Hz vereinheitlicht.** Erststart, Rücksetzen und
   ungültige Einstellungen verwenden dieselbe gerätespezifische Vorgabe;
   Tuner und BFO werden dazu konsistent berechnet. Gültige gespeicherte
   Werte bleiben erhalten, einschließlich eines ausdrücklich gespeicherten 0.
2. **Frequenzzustand sichtbar.** Status zeigt neben angeforderter ZF auch
   Tuner-kHz, konfigurierte ZF aus Tuner/BFO, deren nominales Äquivalent nach
   Kalibrierung und eine manuelle BFO-Abweichung. Dies sind
   Software-Konfigurationswerte, keine unabhängigen RF-Messungen.
3. **Dokumentation/Hilfe/Kommentare bereinigt.** `HANDOVER_PEGEL.md` und
   die RX-Abschnitte von `WIRING.txt` trennen tatsächlichen Aufbau,
   Messungen und Hypothesen. Die Bakenzuordnung und die interne Ursache des
   Referenztaktproblems bleiben offen.
4. **Git-Kennung der Firmware.** `version`, Start und Diagnose ergänzen die
   Versionsnummer/Buildzeit um `git=`, mit `-dirty` bei versionierten
   Änderungen zum Buildzeitpunkt.

## Kalibrierung nur einmal anwenden

Der bisherige Messwert für den Bullseye-LO ist **9750,0043 MHz**. Gegenüber
nominal 9750 MHz kommen Signale 4,3 kHz tiefer in der ZF an. Der Treiber
addiert `rx cal` zum angeforderten Wert: dazu passt **−4300 Hz**.

Beispiel für 10489,540 MHz bei TinySA-Mischer-LO 720,137 MHz:

| `rx cal` [Hz] | `rx freq` [kHz] | konfigurierte physische ZF [kHz] |
|---:|---:|---:|
| 0 | 19398.7 | 19398.7 |
| −4300 | 19403 | 19398.7 |

Die Zeilen sind alternative Einstellungen. Bei bekannter direkter
ZF-Einspeisung `rx cal 0` verwenden. Die −4300 Hz sind ein bisheriger
Messwert; RTL-SDR-/TinySA-Fehler und LNB-Temperatur können beitragen.
Ein GPS-gebundener Mischer-LO allein kalibriert noch nicht den LNB.

## GUI-Korrektur nach Firmware-Flash

Die GUI behandelt einen abgebrochenen USB-Lesethread jetzt als getrennte
Verbindung und verhindert, dass ein alter Thread eine neue Verbindung
beeinflusst. Nach dem Flashen den aktuellen Port neu verbinden. RX-Antworten
(`!R`) erscheinen auch bei ausgeblendeten Push-Meldungen in der Konsole.
Der Versionsparser versteht alte Ausgaben und die neue `git=`-Kennung.

Die Live-Abfrage prüft auch einen zuvor ausgeschalteten Empfänger erneut;
beim Verbinden werden alte RX-Daten verworfen. Eine Altersanzeige macht
ausbleibende Antworten sichtbar. Dafür muss **gui.py aktualisiert werden**;
das reine Flashen einer UF2 ändert die PC-GUI nicht. Die USB-Deskriptoren
und die Firmware-Konsolenbefehle wurden für diese Korrektur nicht geändert.

Automatische GUI-Tests verwenden simulierte serielle Verbindungen. Sie
ersetzen keinen Test mit dem angeschlossenen Pico. Falls auch `version`
und `help` nach erneutem Verbinden unbeantwortet bleiben, sind die
Verbindung und ein möglicher blockierter Firmware-Hauptablauf weiter zu prüfen.

## Bereits zuvor enthaltene Empfangsfunktionen

* Seit `3bb1596`: BFO-Vorzeichen auf `-Rest` korrigiert und der BFO beim
  Laden gespeicherter Einstellungen wiederhergestellt.
* OLED an/aus, eingefrorene Displayaktualisierung, SMPS-PWM-Modus und
  abschaltbares RSSI-Polling als Diagnosemöglichkeiten.
* Optionaler CW-Audiofilter, einstellbares AGC-Ziel, maximale
  Audioverstärkung und Haltezeit.

## Weiterhin vorgeschlagen, noch nicht umgesetzt

* **Direkte Downlink-Eingabe/-Anzeige:** LNB-Nennfrequenz, Mischer-LO und
  additive Kalibrierung eindeutig definieren; den Offset nicht zweimal anwenden.
* **32768 Hz aus PLL A statt PLL B:** vorgeschlagene Rechnung
  `884 MHz / (210 + 12485/16384) / 128 = 32768 Hz`.
  PLL A und der 52-MHz-TX-Ausgang sollen unverändert bleiben. Getrennt
  implementieren und am Gerät prüfen. Die Rechnung beweist weder
  störungsfreien Empfang noch ausreichenden eigenen Mischer-LO-Pegel.
* **Empfangs-Audio als USB-Mikrofon:** bisher besteht der UAC1-Lautsprecherweg
  PC → Gerät. ADC1 → PC ist eine eigene Aufgabe.
* **Versorgung/Analogaufbau untersuchen:** Referenztakt, Bias-Spannung,
  Audioauskopplung, Masseführung und ADC/PWM mit anderen DIY-SSB-Projekten
  vergleichen. Berühren des Teilers verändert den Ton; die Quelle ist noch
  nicht lokalisiert.
* **Baken eindeutig identifizieren:** die Angaben „CW bei 19360 kHz“ und
  später „BPSK bei 19360 kHz“ widersprechen sich. Träger-/Spektrumszuordnung
  bei protokollierten LO-Werten messen.
* **Mögliche unbeabsichtigte Encoder-Abstimmung prüfen:** Kabelbewegungen
  verändern den Ton, erneutes Anklicken der Bake stellt den Empfang wieder her.
  Vor und nach dem Fehler frische `rx`-Antworten vergleichen, bevor erneut
  abgestimmt wird. Auf der OLED-RX-Seite können Encoder-Flanken RX-Tunes
  auslösen; der GUI-RX-Reiter ist davon unabhängig. Ein Fehler durch den
  Encoder ist noch nicht nachgewiesen.

## Keine feste BFO-500-Hz-Vorgabe

`rx bfo 500` machte einen direkt eingespeisten Träger gut hörbar, verändert
aber die Abstimmung. Der BFO wird bereits für den Sub-kHz-Anteil und die
Kalibrierung gebraucht. Deshalb kein pauschaler Zusatzversatz; gewünschte
Tonhöhe/CW-Filter an der tatsächlichen Trägerlage einstellen.
