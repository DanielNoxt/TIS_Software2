# TIS-Software - ReadMe

## Sensor ohne Bootloader flashen
1. [Bootloader Projekt](https://gitea.dclaassen.dedyn.io/TIS/TIS_Bootloader_F446) aufsetzen (Eigene Anleitung) (für Depedencies)
2. Seriennummer und Sensoreigenschaften in SensorInfo.CSV hinterlegen, Softwareversion und Builddatum werden automatisch ausgefüllt
3. Gewünschte Seriennummer in main.h hinterlegen
4. Kompilieren (Debug App only) und flashen

## Sensor mit Bootloader flashen
1. [Bootloader Projekt](https://gitea.dclaassen.dedyn.io/TIS/TIS_Bootloader_F446) aufsetzen (Eigene Anleitung)
2. Seriennummer und Sensoreigenschaften in SensorInfo.CSV hinterlegen, Softwareversion und Builddatum werden automatisch ausgefüllt
3. Gewünschte Seriennummer in main.h hinterlegen und kompilieren (Release)
4. 'SBSFU_DDF_STM32F446VET7.bin' im Überordner 'Binary' mit hilfe von STM32CubeProgrammer flashen

## Firmware Binaries generieren
1. [Bootloader Projekt](https://gitea.dclaassen.dedyn.io/TIS/TIS_Bootloader_F446) aufsetzen
2. Sensoreigenschaften in SensorInfo.CSV für gewünschte Default Seriennummer hinterlegen (z.b. SN2000), Softwareversion und Builddatum werden automatisch ausgefüllt
3. Gewünschte Default Seriennummer in main.h hinterlegen und kompilieren
4. Gewünschte Seriennummern in generate_binaries.py hinterlegen
5. generate_binaries.py ausführen, Binaries sollte sich in Überordner 'Binary' befinden

## Getting Started
### Software:
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) fürs flashen
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html "STM32CubeIDE")  als Entwicklungsumgebung
- Bei Nutzung von Windows: [Git BASH](https://gitforwindows.org "git for windows") zum Ausführen des Prebuild-Shell-Skrips
-  [LibreOffice](https://de.libreoffice.org/ "LibreOffice") zum Editieren der SensorInfo.CSV, da Excel Unterstützung von CSVs mangelhaft ist.