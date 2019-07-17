"""
Not at all completed, but working for average measurements.
Instructions:
    1. SMU_ENABLED should be False in almost any case
    2. WRITE_FIRMWARE should be set to True only if you don't have the PPK with proper firmware
    3. MEASUREMENT_TIME: set this to the number of seconds you need to measure.

The script will spit out the average value after the number of seconds set.
"""
import time

import numpy as np

from ppk.ppk import API as ppkapi
from pynrfjprog import API, Hex


# Draining current with SMU2450? Just keep False if you don't know what this is.
SMU_ENABLED = False

# Flash the board?
WRITE_FIRMWARE = False

# Time to measure in seconds
MEASUREMENT_TIME = 3


def _write_firmware(rtt):
    """Replaces the PPK's firmware."""
    hex_file_path = '.\\hex\\ppk_nrfconnect.hex'
    print("Erasing")
    rtt.erase_all()
    print("Erased")
    application = Hex.Hex(hex_file_path)  # Parsing hex file into segments
    print("Flashing")
    for segment in application:
        print("...")
        rtt.write(segment.address, segment.data, True)
    print("Flashed!")


def _main():
    """Connects to a PPK and prints the average current."""
    board_id = None
    m_time = 0

    if SMU_ENABLED:
        from smu.SMU2450 import API as smuAPI

        smu = smuAPI()

        if smu.discover_and_connect() is False:
            print('Test failed, no device found')

        print("Connected!")

        smu.write('SENS:FUNC "VOLT"')
        smu.output_disable()
        smu.set_source_current()
        smu.set_current_drain_microamp(0)
        smu.output_enable()

    rtt = API.API('NRF52')
    rtt.open()
    rtt.connect_to_emu_without_snr()

    if WRITE_FIRMWARE:
        _write_firmware(rtt)

    ppk = ppkapi(rtt, logprint=False)
    ppk.connect()
    ppk.vdd_set(3000)
    ppk.clear_user_resistors()
    ppk.average_measurement_stop()

    ppk.average_measurement_start()

    board_id = ppk.get_connected_board_id()
    print("PPK Board ID: " + board_id)

    if SMU_ENABLED:
        smu.set_current_drain_microamp(10)
        time.sleep(0.5)

    ppk.average_measurement_start()
    ppk.measurement_readout_start()

    while m_time < MEASUREMENT_TIME:
        time.sleep(1)
        m_time += 1
        print(" Remaining time: {:d}".format(MEASUREMENT_TIME-m_time), end='\r')

    # Omit first 500 samples to avoid any jitter errors on startup.
    result = ppk.avg_buffer[500:]
    ppk.average_measurement_stop()

    print('Average result:')
    print(np.average(result))

    if SMU_ENABLED:
        smu.output_disable()

    rtt.disconnect_from_emu()
    rtt.close()


if __name__ == "__main__":
    _main()
