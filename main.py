"""
Simple CLI for working with the Nordic Power Profiler Kit (PPK).

NOTE: The PPK resets the DUT when python connects. The --power_cycle_dut
      option can be used to add a second DUT reset followed by a delay
      to ensure that the DUT's firmware has time to start up.
"""
import sys
import os
import argparse
import time
import csv
import json

import pandas

from ppk import ppk
import pynrfjprog
from pynrfjprog import API, Hex


HEX_FILE_PATH = os.path.sep.join((".", "hex", "ppk.hex"))


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
    if nrfjprog_api:
        nrfjprog_api.disconnect_from_emu()
        nrfjprog_api.close()
    sys.exit(status)


def _measure_avg(ppk_api, time_s, out_file, draw_png, print_json):
    """Prints the average current over the specified amount of time."""
    avg, timestamped_buf = ppk_api.measure_average(time_s)
    graph_file = None
    csv_file = None
    if out_file:
        csv_file = _replace_file_suffix(out_file, '.csv')
        with open(csv_file, "w", newline='') as data_file:
            csv_writer = csv.writer(data_file)
            csv_writer.writerow(("Timestamp (us)", "Current (uA)"))
            for row in timestamped_buf:
                csv_writer.writerow(row)
        if draw_png:
            graph_file = _replace_file_suffix(out_file, '.png')
            _save_png(avg, timestamped_buf, graph_file)
    if not print_json:
        print('Average: %0.2fuA' % avg)
    else:
        result_dict = {'OPERATION': 'AVERAGE'}
        result_dict['RESULT'] = avg
        result_dict['TIME_S'] = time_s
        if csv_file:
            result_dict['CSV_FILE'] = csv_file
            if graph_file:
                result_dict['GRAPH_FILE'] = graph_file
        print(json.dumps(result_dict))


def _measure_triggers(ppk_api, time_us, level_ua, count, out_file, draw_png, print_json):
    """Acquire and process trigger buffers."""
    json_dict = None
    buffers = ppk_api.measure_triggers(time_us, level_ua, count)
    if print_json:
        json_dict = {'OPERATION': 'TRIGGER'}
        json_dict['TIME_US'] = time_us
        json_dict['LEVEL_UA'] = level_ua
        json_dict['COUNT'] = count
    _process_triggers(buffers, out_file, draw_png, json_dict)


def _measure_ext_triggers(ppk_api, time_us, count, out_file, draw_png, print_json):
    """Acquire and process external trigger buffers."""
    json_dict = None
    buffers = ppk_api.measure_external_triggers(time_us, count)
    if print_json:
        json_dict = {'OPERATION': 'TRIGGER_EXTERNAL'}
        json_dict['TIME_US'] = time_us
        json_dict['COUNT'] = count
    _process_triggers(buffers, out_file, draw_png, json_dict)


def _process_triggers(buffers, out_file, draw_png, json_dict):
    """Save the buffers if necessary and report their averages."""
    if out_file:
        csv_file = _replace_file_suffix(out_file, '.csv')
        if json_dict:
            json_dict['CSV_FILE'] = csv_file
        with open(csv_file, "w", newline='') as data_file:
            csv_writer = csv.writer(data_file)
            csv_writer.writerow(("Timestamp (us)", "Current (uA)"))
            for avg, timestamped_buf in buffers:
                for row in timestamped_buf:
                    csv_writer.writerow(row)
        if draw_png:
            if len(buffers) == 1:
                graph_file = _replace_file_suffix(out_file, '.png')
                avg, data = buffers[0]
                _save_png(avg, data, graph_file)
                if json_dict:
                    json_dict['GRAPH_FILE'] = graph_file
            else:
                graph_files = []
                for i, buffer in enumerate(buffers):
                    graph_file = _replace_file_suffix(out_file, '_%d.png' % i)
                    graph_files.append(graph_file)
                    avg, timestamped_buf = buffer
                    _save_png(avg, timestamped_buf, graph_file)
                if json_dict:
                    json_dict['GRAPH_FILES'] = graph_files
    if len(buffers) == 1:
        result = buffers[0][0]
        if json_dict:
            json_dict['RESULT'] = result
            print(json.dumps(json_dict))
        else:
            print("Trigger buff average: %0.2fuA" % result)
    else:
        results = []
        for i, buffer in enumerate(buffers):
            avg, timestamped_buf = buffer
            if json_dict:
                results.append(avg)
            else:
                print("Trigger buff %d average: %0.2fuA" % (i, avg))
        if json_dict:
            json_dict['RESULTS'] = results
            print(json.dumps(json_dict))


def _save_png(avg, data, out_file):
    """Takes a sequence of (timestamp, measurement) rows and saves a simple
    line graph with the given file_path.
    """
    labels = ['Timestamp (us)', ('Current (uA)\nAVG: %0.2fuA' % avg)]
    data_frame = pandas.DataFrame.from_records(data, columns=labels)
    lines = data_frame.plot.line(x=labels[0], y=labels[1])
    lines.yaxis.grid(True, linestyle='--')
    fig = lines.get_figure()
    fig.savefig(out_file)


def _replace_file_suffix(file_name, suffix):
    """Replace the .XXX suffix in file_name with suffix."""
    return os.path.splitext(file_name)[0] + suffix


def _connect_to_emu(args):
    """Connects to emulator and replaces the PPK firmware if necessary."""
    nrfjprog_api = pynrfjprog.API.API('NRF52')
    nrfjprog_api.open()

    if args.serial_number:
        nrfjprog_api.connect_to_emu_with_snr(args.serial_number)
    else:
        nrfjprog_api.connect_to_emu_without_snr()

    if not args.skip_verify:
        fw_hex = pynrfjprog.Hex.Hex(HEX_FILE_PATH)
        if not _verify_firmware(nrfjprog_api, fw_hex):
            if args.force:
                _write_firmware(nrfjprog_api, fw_hex)
            else:
                print("PPK firmware verification failed. Use -f option to replace it.")
                _close_and_exit(nrfjprog_api, -1)
    return nrfjprog_api


def _add_and_parse_args():
    """Build the argparse object and parse the args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--serial_number", type=int,
                        help="serial number of J-Link")
    parser.add_argument("-a", "--average", type=float,
                        help="print average current over time")
    parser.add_argument("-w", "--trigger_microseconds", type=int, nargs='?', default=5850,
                        help="set trigger window in microseconds [%d, %d]" %
                        (ppk.API.TRIG_WINDOW_MIN_US, ppk.API.TRIG_WINDOW_MAX_US))
    parser.add_argument("-n", "--trigger_count", type=int, nargs='?', default=1,
                        help="set number of trigger buffers to capture")
    parser.add_argument("-e", "--external_vdd", type=int,
                        help="set external regulator voltage [%d, %d]" %
                        (ppk.API.EXT_REG_MIN_MV, ppk.API.EXT_REG_MAX_MV))
    parser.add_argument("-c", "--clear_user_resistors",
                        help="clear user calibration resistors", action="store_true")
    parser.add_argument("-p", "--power_cycle_dut",
                        help="power cycle the DUT and delay", nargs='?', const=0, type=float)
    parser.add_argument("-o", "--out_file",
                        help="write measurement data to file", type=str)
    parser.add_argument("-z", "--png",
                        help="create .png graph(s) of data in out_file", action="store_true")
    parser.add_argument("-g", "--spike_filtering",
                        help="enable spike filtering", action="store_true")
    trig_group = parser.add_mutually_exclusive_group()
    trig_group.add_argument("-t", "--trigger_microamps", type=int,
                            help="set trigger threshold in microamps")
    trig_group.add_argument("-x", "--enable_ext_trigger",
                            help="enable 'TRIG IN' external trigger", action="store_true")
    fw_group = parser.add_mutually_exclusive_group()
    fw_group.add_argument("-k", "--skip_verify",
                          help="save time by not verifying the PPK firmware",
                          action="store_true")
    fw_group.add_argument("-f", "--force",
                          help="program the PPK firmware if necessary",
                          action="store_true")
    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument("-v", "--verbose",
                              help="print logging information", action="store_true")
    output_group.add_argument("-j", "--json",
                              help="print output as parsable JSON",
                              action="store_true")
    args = parser.parse_args()
    if not args.trigger_microamps and not args.enable_ext_trigger:
        if not args.average:
            parser.print_usage()
            print("main.py: error: no measurement operation specified")
            sys.exit(-1)
    if args.trigger_microseconds:
        if (args.trigger_microseconds < ppk.API.TRIG_WINDOW_MIN_US or
                args.trigger_microseconds > ppk.API.TRIG_WINDOW_MAX_US):
            parser.print_usage()
            print("main.py: error: invalid trigger window width (%d)" %
                  args.trigger_microseconds)
            sys.exit(-1)
    if args.external_vdd:
        if args.external_vdd < ppk.API.EXT_REG_MIN_MV or args.external_vdd > ppk.API.EXT_REG_MAX_MV:
            parser.print_usage()
            print("main.py: error: invalid external voltage regulator value (%d)" %
                  args.external_vdd)
            sys.exit(-1)
    if args.png:
        if not args.out_file:
            parser.print_usage()
            print("main.py: error: out_file required to when creating .png graph(s))")
            sys.exit(-1)
    return args


def _main():
    """Parses arguments for the PPK CLI."""
    args = _add_and_parse_args()
    nrfjprog_api = None
    try:
        nrfjprog_api = _connect_to_emu(args)

        ppk_api = ppk.API(nrfjprog_api, logprint=args.verbose)
        ppk_api.connect()

        if args.external_vdd:
            ppk_api.set_external_reg_vdd(args.external_vdd)

        if args.clear_user_resistors:
            ppk_api.clear_user_resistors()

        if args.power_cycle_dut is not None:
            ppk_api.disable_dut_power()
            ppk_api.enable_dut_power()
            if args.power_cycle_dut:
                if args.verbose:
                    print("Sleep for %0.1fs." % args.power_cycle_dut)
                time.sleep(args.power_cycle_dut)

        if args.spike_filtering:
            ppk_api.enable_spike_filtering()

        if args.average:
            _measure_avg(ppk_api,
                         args.average,
                         args.out_file,
                         args.png,
                         args.json)

        if args.trigger_microamps:
            _measure_triggers(ppk_api,
                              args.trigger_microseconds,
                              args.trigger_microamps,
                              args.trigger_count,
                              args.out_file,
                              args.png,
                              args.json)
        elif args.enable_ext_trigger:
            _measure_ext_triggers(ppk_api,
                                  args.trigger_microseconds,
                                  args.trigger_count,
                                  args.out_file,
                                  args.png,
                                  args.json)

        _close_and_exit(nrfjprog_api, 0)
    except Exception as ex:
        print("main.py: error: " + str(ex))
        _close_and_exit(nrfjprog_api, -1)


if __name__ == "__main__":
    _main()
