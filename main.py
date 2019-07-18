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

from ppk import ppk
import pynrfjprog
from pynrfjprog import API, Hex


HEX_FILE_PATH = os.path.sep.join((".", "hex", "ppk_nrfconnect.hex"))


def _verify_firmware(nrfjprog_api, fw_hex):
    """"""
    for segment in fw_hex:
        content = nrfjprog_api.read(segment.address, len(segment.data))
        if segment.data != content:
            return False
    return True


def _write_firmware(nrfjprog_api, fw_hex):
    """Replaces the PPK's firmware."""
    print("Replacing PPK firmware...", end='')
    nrfjprog_api.erase_all()
    for segment in fw_hex:
        nrfjprog_api.write(segment.address, segment.data, True)
    print("done")


def _close_and_exit(nrfjprog_api, status):
    """"""
    nrfjprog_api.disconnect_from_emu()
    nrfjprog_api.close()
    sys.exit(status)


def _measure_avg(ppk_api, time_s):
    """Prints the average current over the specified amount of time."""
    board_id = None
    m_time = 0

    ppk_api.average_measurement_start()

    # TODO: This is suspicious. Maybe it's just a delay?
    board_id = ppk_api.get_connected_board_id()
    print("PPK Board ID: " + board_id)

    ppk_api.average_measurement_start()
    ppk_api.measurement_readout_start()

    while m_time < time_s:
        time.sleep(1)
        m_time += 1
        print(" Remaining time: {:d}".format(time_s-m_time), end='\r')

    # Omit first 500 samples to avoid any jitter errors on startup.
    result = ppk_api.avg_buffer[500:]
    ppk_api.average_measurement_stop()

    print('Average result:')
    print(np.average(result))


def _set_trigger(ppk_api, voltage):
    """Prints the average current after the trigger voltage is reached."""
    pass


def _main():
    """Parses arguments for the PPK CLI."""
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--force",
                        help="program the PPK firmware if necessary",
                        action="store_true")
    parser.add_argument("-s", "--serial_number", type=int,
                        help="serial number of J-Link")
    parser.add_argument("-a", "--average", type=int,
                        help="print average current over time")
    parser.add_argument("-t", "--trigger_voltage", type=int,
                        help="print average current after trigger")
    parser.add_argument("-v", "--vdd", type=int,
                        help="set external regulator voltage [2100, 3600]")
    args = parser.parse_args()

    if not args.trigger_voltage and not args.average:
        parser.print_usage()
        sys.exit(-1)

    nrfjprog_api = pynrfjprog.API.API('NRF52')
    nrfjprog_api.open()
    nrfjprog_api.connect_to_emu_without_snr()

    fw_hex = pynrfjprog.Hex.Hex(HEX_FILE_PATH)
    if not _verify_firmware(nrfjprog_api, fw_hex):
        if args.force:
            _write_firmware(nrfjprog_api, fw_hex)
        else:
            print("PPK firmware verification failed. Use -f option to replace it.")
            _close_and_exit(nrfjprog_api, -1)

    ppk_api = ppk.API(nrfjprog_api, logprint=False)
    ppk_api.connect()

    if args.vdd:
        if args.vdd < ppk.VDD_SET_MIN or args.vdd > ppk.VDD_SET_MAX:
            print("Invalid external voltage regulator value (%d)." % args.vdd)
            _close_and_exit(nrfjprog_api, -1)
        else:
            ppk_api.vdd_set(args.vdd)

    ppk_api.clear_user_resistors()
    ppk_api.average_measurement_stop()

    if args.trigger:
        _set_trigger(ppk_api, args.trigger)

    if args.average:
        _measure_avg(ppk_api, args.average)

    _close_and_exit(nrfjprog_api, 0)


if __name__ == "__main__":
    _main()
