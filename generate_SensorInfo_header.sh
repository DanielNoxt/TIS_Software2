#!/bin/bash

set -e  # Beenden Sie das Skript, wenn ein Befehl fehlschlägt

# Wechseln Sie zum Verzeichnis, in dem das Skript sich befindet
cd "$(dirname "$0")" || exit

# Prüfen Sie, ob wir im richtigen Verzeichnis sind
if [ ! -d ".git" ] && [ ! -f ".git" ] ; then
    echo "This script must be run from the root directory of the Git repository."
    exit 1
fi

# Holen Sie sich das neueste Tag aus Git
GIT_TAG=$(git describe --tags HEAD)

# Überprüfen, ob ein Tag gefunden wurde
if [ -z "$GIT_TAG" ]; then
    echo "No tags found for the current commit."
    exit 1
fi

# Entfernen Sie das 'v' Präfix
CLEAN_TAG=${GIT_TAG#v}

# Split das Tag in Major.Minor.Patch
IFS='.' read -r SW_MAJOR SW_MINOR SW_PATCH <<< "$CLEAN_TAG"

# Versionsnummer ausgeben (für postbuild script)
printf "$SW_MAJOR $SW_MINOR $SW_PATCH" > output.txt

# Extrahieren Sie die SENSOR_SN aus der main.h Datei
SENSOR_SN=$(grep '#define SENSOR_SN' Core/Inc/main.h | awk '{print $3}')

# Überprüfen, ob die SENSOR_SN gefunden wurde
if [ -z "$SENSOR_SN" ]; then
    echo "SENSOR_SN not found in main.h"
    exit 1
fi

echo "Found SENSOR_SN: $SENSOR_SN"

# Lesen Sie die Parameter aus der CSV-Datei (mit Komma als Trennzeichen)
LINE=$(awk -F',' -v sn="$SENSOR_SN" '$1 == sn {print $0}' SensorInfo.CSV)
IFS=',' read -r HEADER HW_VERSION SW_VERSION BUILD_DATE BUILD_TIME TIME_SEND_LOCK_IN_DATA TIME_SEND_ADDITIONAL_DATA TIME_SEND_RAW_DATA TIME_SEND_META_DATA AVERAGES_LOCK_IN \
PRESET0_FREQ G0_F0 G1_F0 G2_F0 G3_F0 P0_F0 P1_F0 P2_F0 P3_F0 \
PRESET1_FREQ G0_F1 G1_F1 G2_F1 G3_F1 P0_F1 P1_F1 P2_F1 P3_F1 \
PRESET2_FREQ G0_F2 G1_F2 G2_F2 G3_F2 P0_F2 P1_F2 P2_F2 P3_F2 <<< "$LINE"

# Überprüfen, ob eine Hardware-Version gefunden wurde
if [ -z "$HW_VERSION" ]; then
    echo "Hardware version for SENSOR_SN $SENSOR_SN not found in CSV file"
    echo "Available SENSOR_SN in CSV:"
    awk -F',' '{print $1}' SensorInfo.CSV
    exit 1
fi

echo "Found HW_VERSION: $HW_VERSION"

# Split die Hardware-Version in Major.Minor.Patch
IFS='.' read -r HW_MAJOR HW_MINOR HW_PATCH <<< "${HW_VERSION#v}"

# Stellen Sie sicher, dass das Verzeichnis existiert
mkdir -p Core/Inc

# Ermitteln Sie das aktuelle Datum und die Uhrzeit
BUILD_DATE=$(date +'%Y-%m-%d')
BUILD_TIME=$(date +'%H:%M:%S')

# Erstellen Sie die SensorInfo.h Datei
cat <<EOL > Core/Inc/SensorInfo.h
#ifndef SENSOR_INFO_H
#define SENSOR_INFO_H

// Automatisch erzeugt durch generate_version_header.sh als PreBuild

#define SW_VERSION_MAJOR $SW_MAJOR
#define SW_VERSION_MINOR $SW_MINOR
#define SW_VERSION_PATCH $SW_PATCH

#define HW_VERSION_MAJOR $HW_MAJOR
#define HW_VERSION_MINOR $HW_MINOR
#define HW_VERSION_PATCH $HW_PATCH

#define TIME_SEND_LOCK_IN_DATA (uint32_t)$TIME_SEND_LOCK_IN_DATA // Periode in ms
#define TIME_SEND_ADDITIONAL_DATA (uint32_t)$TIME_SEND_ADDITIONAL_DATA // Periode in ms
#define TIME_SEND_RAW_DATA (uint32_t)$TIME_SEND_RAW_DATA // Periode in ms
#define TIME_SEND_META_DATA (uint32_t)$TIME_SEND_META_DATA // Periode in ms

#define AVERAGES_LOCK_IN $AVERAGES_LOCK_IN

#define PRESET0_FREQ $PRESET0_FREQ
#define PRESET0_GAIN {$G0_F0,$G1_F0,$G2_F0,$G3_F0}
#define PRESET0_PHASE {$P0_F0,$P1_F0,$P2_F0,$P3_F0}

#define PRESET1_FREQ $PRESET1_FREQ
#define PRESET1_GAIN {$G0_F1,$G1_F1,$G2_F1,$G3_F1}
#define PRESET1_PHASE {$P0_F1,$P1_F1,$P2_F1,$P3_F1}

#define PRESET2_FREQ $PRESET2_FREQ
#define PRESET2_GAIN {$G0_F2,$G1_F2,$G2_F2,$G3_F2}
#define PRESET2_PHASE {$P0_F2,$P1_F2,$P2_F2,$P3_F2}

#endif // SENSOR_INFO_H
EOL

# Erstellen oder aktualisieren Sie die CSV-Datei mit der SW-Version und dem Build-Datum und -Uhrzeit
awk -F',' -v sn="$SENSOR_SN" -v sw="v$SW_MAJOR.$SW_MINOR.$SW_PATCH" -v date="$BUILD_DATE" -v time="$BUILD_TIME" '
BEGIN { OFS="," }
$1 == sn { $3 = sw; $4 = date; $5 = time }
{ print }
' SensorInfo.CSV > SensorInfo_temp.CSV && mv SensorInfo_temp.CSV SensorInfo.CSV