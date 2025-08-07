import os
import re

sensor_sn_ref = 2000
sensor_sn_first = 2000
sensor_sn_last  = 2003
ID_OFFSET = 49152

def calculate_can_id(sensor_sn):
    return (sensor_sn - sensor_sn_ref) * 65536 + 483393774

if __name__ == "__main__":

	path = "../Binary/SBSFU_DDF_STM32F446VET7.bin"
	with open(path, "rb") as file:
		original_binary = file.read()

	for sensor_sn in range(sensor_sn_first, sensor_sn_last + 1):
		filename = f"../Binary/SN{sensor_sn}.bin"
		with open(filename, "wb") as new_file:
			new_file.write(original_binary)
			new_file.seek(ID_OFFSET)
			new_ID = calculate_can_id(sensor_sn).to_bytes(4, byteorder='little', signed=False)
			new_file.write(new_ID)

