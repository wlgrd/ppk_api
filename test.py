# test regex
# regex lib

import re
from ppk.ppk import API as ppkapi
from smu.SMU2450 import API as smuAPI
from pynrfjprog import API, Hex
import time

board_id = None
hex_file_path = 'ppk_nrfconnect.hex'

rtt = API.API('NRF52')
rtt.open()
rtt.connect_to_emu_without_snr()

# smu = smuAPI()

# if smu.discover_and_connect() is False:
#     print('Test failed, no device found')

# print("Connected!")

# print("Erasing")
# rtt.erase_all()
# print("Erased")
# application = Hex.Hex(hex_file_path)  # Parsing hex file into segments
# print("Flashing")
# for segment in application:
#     print("...")
#     rtt.write(segment.address, segment.data, True)
# print("Flashed!") 

ppk = ppkapi(rtt, logprint=True)
ppk.connect()
ppk.vdd_set(3000)
ppk.clear_user_resistors()
ppk.average_measurement_stop()

# smu.write('SENS:FUNC "VOLT"')
# smu.output_disable()
# smu.set_source_current()
# smu.set_current_drain_microamp(3)
# smu.output_enable()
ppk.average_measurement_start()

# print(smu.read_voltage()) # need to fix this... 
board_id = ppk.get_connected_board_id()
print("PPK Board ID: " + board_id)

ppk.measurement_readout_start()

time.sleep(5)


