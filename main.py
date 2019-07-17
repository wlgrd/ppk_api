"""
TODO: Verify the PPK firmware and exit if it fails unless a --f is provided.
      For printing an average lets have a configurable delay and sample length
      For trigger we should have voltage and sample length.
      Stretch goal is to have an option for outputting a png graph.
"""
import time

import numpy as np

from ppk.ppk import API as ppkapi
from pynrfjprog import API, Hex


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

    rtt.disconnect_from_emu()
    rtt.close()


if __name__ == "__main__":
    _main()
