# This Python file uses the following encoding: utf-8
import os
import re

# Definieren der Konstanten
SensorSN_Ref = 2000  # Ausgangsdatei
SensorSN_First = 2000  # erste SN, für die eine DBC erstellt werden soll
SensorSN_Last = 2025  # letzte SN, für die eine DBC erstellt werden soll.
SENSORSN_SHIFT = 65536
DATA_ID_SHIFT = 256
DATASET_OFFSET = 483393774
DBC_EXT_ID_INDICATOR = 2147483648  # 32stes Bit als Indikator für Ext ID

def calculate_can_id(sensor_sn, data_id):
    return (sensor_sn - SensorSN_Ref) * SENSORSN_SHIFT + data_id * DATA_ID_SHIFT + DATASET_OFFSET + DBC_EXT_ID_INDICATOR

def replace_can_ids(file_content, old_sensor_sn, new_sensor_sn):
    new_content = file_content
    for data_id in range(256):
        old_can_id = calculate_can_id(old_sensor_sn, data_id)
        new_can_id = calculate_can_id(new_sensor_sn, data_id)
        # Verwenden Sie einen regulären Ausdruck, um sicherzustellen, dass nur ganze IDs ersetzt werden
        new_content = re.sub(rf'\b{old_can_id}\b', str(new_can_id), new_content)
    return new_content

def replace_sn_references(file_content, old_sensor_sn, new_sensor_sn):
    old_sn_str = f"TIS_SN{old_sensor_sn}"
    new_sn_str = f"TIS_SN{new_sensor_sn}"
    new_content = file_content.replace(old_sn_str, new_sn_str)

    old_sn_str_short = f"SN{old_sensor_sn}"
    new_sn_str_short = f"SN{new_sensor_sn}"
    new_content = new_content.replace(old_sn_str_short, new_sn_str_short)
    
    return new_content

def process_files(sensor_sn_ref, sensor_sn_first, sensor_sn_last):
    input_filename = f"SN{sensor_sn_ref}.dbc"
    if not os.path.exists(input_filename):
        print(f"Datei {input_filename} existiert nicht.")
        return
    
    # Datei einlesen
    with open(input_filename, "r") as file:
        file_content = file.read()
    
    # Dateien für die Seriennummern von sensor_sn_first bis sensor_sn_last erstellen
    all_content = ""
    for sensor_sn in range(sensor_sn_first, sensor_sn_last + 1):
        new_content = replace_can_ids(file_content, sensor_sn_ref, sensor_sn)
        new_content = replace_sn_references(new_content, sensor_sn_ref, sensor_sn)
        new_filename = f"SN{sensor_sn}.dbc"
        with open(new_filename, "w") as new_file:
            new_file.write(new_content)
        print(f"Datei {new_filename} wurde erstellt.")
        all_content += "\n\n" + new_content

    # Erstelle die DBC-Datei mit allen Inhalten
    all_filename = f"SN{sensor_sn_first}_SN{sensor_sn_last}.dbc"
    with open(all_filename, "w") as all_file:
        all_file.write(all_content)
    print(f"Datei {all_filename} wurde erstellt.")

def delete_existing_dbc_files(ref_filename):
    try:
        ref_filename = f"SN{SensorSN_Ref}.dbc"
        for filename in os.listdir():
            if filename.endswith(".dbc") and filename != ref_filename:
                os.remove(filename)
        print(f"Ueberfluessige Dateien wurden geloescht.")
    except Exception as e:
        print(f"Fehler beim Loeschen der Dateien: {e}")

if __name__ == "__main__":
    delete_existing_dbc_files(SensorSN_Ref)
    process_files(SensorSN_Ref, SensorSN_First, SensorSN_Last)
