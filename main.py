"""
Not at all completed, but working for average measurements.
Instructions:
    1. write_firmware should be set to True only if you don't have the PPK with proper firmware
    2. measurement_time: set this to the number of seconds you need to measure.

The script will spit out the average value after the number of seconds set.
"""

from ppk.ppk import API as ppkapi
from pynrfjprog import API, Hex
import time
import numpy as np

# Flash the board?
write_firmware = False
hex_file_path = '.\\hex\\ppk_v2.0.1rc.hex'

# Time to measure in seconds
measurement_time = 3

board_id = None
m_time = 0

print("Starting...")
rtt = API.API('NRF52')
rtt.open()
rtt.connect_to_emu_without_snr()

if(write_firmware):
    print("Erasing")
    rtt.erase_all()
    print("Erased")
    application = Hex.Hex(hex_file_path)  # Parsing hex file into segments
    print("Flashing")
    for segment in application:
        print("...")
        rtt.write(segment.address, segment.data, True)
    print("Flashed!")

ppk = ppkapi(rtt, logprint=False)
ppk.connect()
print("Setting vdd")
ppk.vdd_set(2800)
time.sleep(1)
ppk.clear_user_resistors()

ppk.average_measurement_stop()
ppk.average_measurement_start()

board_id = ppk.get_connected_board_id()
print("PPK Board ID: " + board_id)



ppk.average_measurement_start()
ppk.measurement_readout_start()

while(m_time < measurement_time):
    time.sleep(1)
    m_time += 1
    # Removed due to using python 2.7.12
    #print(" Remaining time: {:d}".format(measurement_time-m_time),end='\r')

# Omit first 500 samples to avoid any jitter errors on startup.
result = ppk.avg_buffer[500:]
ppk.average_measurement_stop()

print('Average result:')
print(np.average(result))

rtt.disconnect_from_emu()
rtt.close()



