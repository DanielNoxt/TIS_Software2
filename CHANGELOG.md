# TIS-Software - Changelog

## [2.1.5] - Unreleased
### Added
- Phase AutoZero in Software hinzufügen
-- Dafür ausgabe des Korrekturwertes nötig
- Sensoinfo.h umbauen damit variablen in CONSTANTS gespeichert sind, um automatische generierung von Binaries zu ermöglichen
### Fixed
- DBC muss für die einzelnen Bauds erweitert werden. Weiterhin prüfen, 8 bit befehle ok

## [2.1.4] - 30.03.2025
### Added
- Ausgabe der Mittlungen
### Changed
- Watchdog Timeout auf 30 Sekunden verlängert
- MeasMode Frequenzsweeps:
-- Sweep-Logik verbessert: Auflösung des DDS-Chips wird jetzt berücksichtigt, damit keine Frequenzen ausversehen doppelt gemessen werden.

## [2.1.3] - 28.03.2025
### Fixed
- MeasMode Frequenzsweeps:
-- Namenskollision in DBC korrigiert
-- Kollision mit alter Sweepfunktion gefixt

## [2.1.2] - 23.03.2025
### Added
- MeasModes hinzugefügt
-- Normaler MeasMode und Frequenzsweeps implementiert

## [2.1.1] - 14.01.2025
### Changed
- CONSTANTS typdef geändert und delete flag hinzugefügt damit DATA nur optional gelöscht wird
- CAN_Botschaften und CAN_Commands_Toolbox update

## [2.1.0] - datum
### Added 
- Watchdog
- Eigenen Bootloader
- Flash treiber hinzugefügt
- CONSTANTS section hinzugefügt für persistente variablen
- Tool um firmware images mit vorprogrammierten CanIDs zu generieren
- Neues Buildprofil hinzugefügt um mit und ohne Bootloader debuggen zu können
### Changed
- Es wird nun über den generellen Flashtreiber daten in Flash geschrieben
- Baudrate ist nun persistent auch nach updaten
- Flash section Layout geändert für bootloader
- CanID wird nicht mehr beim kompilieren vorgegeben sondern beim Flashen
- Projekt kann als submodul genutzt werden für bootloader projekt
- Update läuft über eigenen bootloader + tool
- Optionbytes entfernt da bootloader diese schon konfiguriert

## [2.0.8] - 2024-07-31
### Added
- Vierter Temperatursensor als Referenz implementiert

## [2.0.7] - Existiert nicht

## [2.0.6] - 2024-07-31
### Added
- Read Out Protection gesetzt, damit ein Auslesen des Programmspeichers über Bootloader oder JTAG unterbunden wird.
- Übertragen der Ontime bzw. Systick über CAN
### Changed
- CAN-Bootloader mit Keys for ungewünschten Zugriff geschützt, da über diesen auch ein Auslesen des Codes möglich ist.
- StateMachine um sendMetaData erweitert, damit Sendefrequenz reduziert werden kann.
- Überprüfung der CAN-Nachrichten auf DLC auskommentiert, CANoe bzw. DBCs haben für Commands keine variable Länge.
### Fixed
- Anpassung an der DBC, damit Frequenz und GAIN für die Anregung einfacher Eingestellt werden können. Vorherige DBC hat dazu geführt, dass bei CLAAS immer der GAIN für einen Kompensationschannel anstelle der Anregung eingestellt wurde.


## [2.0.5] - 2024-07-05
### Added
- Mittelwert (über CAN einstellbare Anzahl an Mittlungen) für alle Channel
-- Initiale Anzahl der Mittlungen aus SensorInfo.CSV
- Mappingtabelle um den zugehörigen DDS-Channel zu einer Elektrode zu bestimmen
### Changed
- Lock_in Ergebnisse von den Referenzdaten getrennt.
- Voreinstellung, welche Elektroden gesampled werden sollen, jetzt als globales 2D-Array
-- const ElecTypeDef SampleElec[Preset][SampleElektroden]
-- Übergabe der Elektroden an Samplefunktionen, zur Vermeidung von Chaos von bei Änderungen
- DBC Converter angepasst, so dass zusätzlich eine StartId eingestellt werden kann
### Fixed
- Weitere Variablen, die über CAN verändert werden, als volatile deklariert. 


## [2.0.4] - 2024-07-01
### Added
- AutoZero Funktion
-- Symmetrischer Moving Average um Phasen- und Frequenzminimum trotz Noise zu finden
- CAN_Commands_Toolbox.CSV, welche die CAN-Befehle für die Sensoren beinhaltet, hinzugefügt
-- Zur µC-Software passende Befehle in der Toolbox vermeiden Kompabilitätsprobleme
### Changed
- Prebuild: Zusätzlich wird GAIN und PHASE für jedes Preset aus SensorInfo.CSV gelesen


## [2.0.3] - 2024-06-25
### Added
- Factory Reset als Softwareoption hinzugefügt
- Sensor gibt jetzt Rückmeldung bei erfolgreichen Parameteränderungen über CAN.
### Changed
- SensorInfo.csv Spaltenreihenfolge angepasst, Seperationszeichen auf "," umgestellt
- Sweep um Gain und Phase erweitert
- ErrorHandling aus der StateMachine entfernt: Funktion SoftError_Handler() soll bei Softwarefehlern aufgerufen werden
- Anpassungen an den CAN-Botschaften und der DBC:
-- Übertragen von Gain und Phase in einer Botschaft, je Channel und Frequenzpreset
-- Übertragen von UID, SensorID, HID und SID jetzt möglich
-- Command-Botschaften als Multiplexed Nachricht in der DBC hinterlegt


## [2.0.2] - 2024-06-17
### Added
- Implementierung eines Prebuild-Skripts zum automatischen Generieren der `SensorInfo.h`-Datei.
-- Extrahiert Software-Versionsinformationen aus Git-Tags und speichert sie in `SensorInfo.h`.
-- Liest Hardware-Versionsinformationen, Timings für LockIn, Additional Data, Raw Data sowie die Frequenz für alle 3 Frequenzpresets aus `SensorInfo.CSV` und speichert sie in `SensorInfo.h`.
-- Aktualisiert die `SW_VERSION` in der `SensorInfo.CSV`, Tabelle dient der Dokumentierung der Voreinstellung aller Sensoren
- CAN-Übertragung von HW-Version und SW_Version im neuen vMajor,Minor,Patch Format


## [2.0.1] - 2024-06-16
### Added
- Sweep Funktion
-- Frequenz kann zwischen Start und Stop mit einer definierten Stepsize ausführt werden.
- Auslesen des GIT-VersionsTags und anschließendes Erzeugen einer .h Datei mit den Versionsinformationen
### Fixed
- Sweep Stepzise wird jetzt auf richtiges Vorzeichen geprüft, zur Vermeidung von Endlosschleifen.


## [2.0.0] - 2024-06-14
### Added
- Bibliotheken zur Sinusanregung mittels DSS (AD9106) erstellt
-- Für jedes Frequenzpreset werden die nötigen Variablen, beispielsweise das TuningWord für die Frequenz, auf dem Controller gespeichert
-- Frequenz, Gain und Phase für alle DSS-Channel über CAN einstellbar
- Sweep Funktion
-- Frequenz kann zwischen Start und Stop mit einer definierten Stepsize ausführt werden
### Fixed
- Jitter-Bug behoben: Deaktivieren aller Interrups zwischen DDS-Trigger und das Starten des ADCs 
