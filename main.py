"""
TODO: Verify the PPK firmware and exit if it fails unless a --f is provided.
      For printing an average lets have a configurable delay and sample length
      For trigger we should have voltage and sample length.
      Stretch goal is to have an option for outputting a png graph.
"""
import sys
import os
import time
import argparse

import numpy as np

from ppk.ppk import API as ppkapi
from pynrfjprog import API, Hex


HEX_FILE_PATH = os.path.sep.join((".", "hex", "ppk_nrfconnect.hex"))

# Flash the board?
WRITE_FIRMWARE = False


def _verify_firmware(nrfjprog, fw):
    """"""
    for segment in fw:
        content = nrfjprog.read(segment.address, len(segment.data))
        if not segment.data == content:
            return False
    return True


def _write_firmware(nrfjprog, fw):
    """Replaces the PPK's firmware."""
    print("Replacing PPK firmware...", end='')
    nrfjprog.erase_all()
    for segment in fw:
        nrfjprog.write(segment.address, segment.data, True)
    print("done")


def _close_and_exit(nrfjprog, status):
    """"""
    nrfjprog.disconnect_from_emu()
    nrfjprog.close()
    sys.exit(status)


def _measure_avg(ppk, time_s):
    """Prints the average current over the specified amount of time."""
    board_id = None
    m_time = 0

    ppk.average_measurement_start()

    # TODO: This is suspicious. Maybe it's just a delay?
    board_id = ppk.get_connected_board_id()
    print("PPK Board ID: " + board_id)

    ppk.average_measurement_start()
    ppk.measurement_readout_start()

    while m_time < time_s:
        time.sleep(1)
        m_time += 1
        print(" Remaining time: {:d}".format(time_s-m_time), end='\r')

    # Omit first 500 samples to avoid any jitter errors on startup.
    result = ppk.avg_buffer[500:]
    ppk.average_measurement_stop()

    print('Average result:')
    print(np.average(result))


def _main():
    """Connects to a PPK and prints the average current."""
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--force",
                         help="program the PPK firmware if necessary",
                         action="store_true")
    parser.add_argument("-id", "--serial_number", type=int,
                         help="serial number of J-Link")
    parser.add_argument("-a", "--average", type=int,
                         help="print average current over time")
    parser.add_argument("-t", "--trigger_voltage", type=int,
                         help="print average current after trigger")
    args = parser.parse_args()

    if args.trigger_voltage:
        print("Print trigger!")

    if args.average:
        print("Print average!")

    if not args.trigger_voltage and not args.average:
        parser.print_usage()
        sys.exit(-1)

    nrfjprog = API.API('NRF52')
    nrfjprog.open()
    nrfjprog.connect_to_emu_without_snr()

    fw = Hex.Hex(HEX_FILE_PATH)
    if not _verify_firmware(nrfjprog, fw):
        if args.force:
            _write_firmware(nrfjprog, fw)
        else:
            print("PPK firmware verification failed. Use -f option to replace it.")
            _close_and_exit(nrfjprog, -1)

    ppk = ppkapi(nrfjprog, logprint=False)
    ppk.connect()
    ppk.vdd_set(3000)
    ppk.clear_user_resistors()
    ppk.average_measurement_stop()

    if args.average:
        _measure_avg(ppk, args.average)

    _close_and_exit(nrfjprog, 0)


if __name__ == "__main__":
    _main()
