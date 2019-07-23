Nordic Semiconductor's [Power Profiler Kit (PPK)](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/Power-Profiler-Kit) is very useful for measuring and optimizing power consumption when developing battery-powered devices. Although the [nRF Connect for Desktop](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Connect-for-desktop) application provides a friendly GUI, it has limited support for logging measurement data and isn't accessible from a command line. The purpose of this library is to make the PPK more useful for logging and integration into automated testing/continuous integration systems.

### Features
The main features of the ppk_api include:

 - Comparable functionality to the nRF Connect GUI
 - Comma Separated Value (CSV) log files for both average and trigger bufer measurement data
 - Written in Python3 for easy integration with other CLI tools
 - Cross platform support

### Requirements
The interface to the PPK requires Nordic's [nRF Command Line Tools](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Command-Line-Tools).

The excellent [nRF Pynrfjprog](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Pynrfjprog) and [numpy](https://numpy.org/) modules can be installed from the command line using pip:
```
$ cd ppk_api
$ pip3 install --user -r requirements.txt
```

### Usage
See **main.py** for an example of how to connect to a PPK and use the API directly. Otherwise, help is available from the command line:

```
$ python3 main.py --help
usage: main.py [-h] [-s SERIAL_NUMBER] [-a AVERAGE] [-t TRIGGER_MICROAMPS]
               [-w [TRIGGER_MICROSECONDS]] [-n [TRIGGER_COUNT]]
               [-e EXTERNAL_VDD] [-c] [-p [POWER_CYCLE_DUT]] [-v]
               [-o OUT_FILE] [-g] [-x] [-k | -f]

optional arguments:
  -h, --help            show this help message and exit
  -s SERIAL_NUMBER, --serial_number SERIAL_NUMBER
                        serial number of J-Link
  -a AVERAGE, --average AVERAGE
                        print average current over time
  -t TRIGGER_MICROAMPS, --trigger_microamps TRIGGER_MICROAMPS
                        set trigger threshold in microamps
  -w [TRIGGER_MICROSECONDS], --trigger_microseconds [TRIGGER_MICROSECONDS]
                        set trigger window in microseconds
  -n [TRIGGER_COUNT], --trigger_count [TRIGGER_COUNT]
                        set number of trigger buffers to capture
  -e EXTERNAL_VDD, --external_vdd EXTERNAL_VDD
                        set external regulator voltage [2100, 3600]
  -c, --clear_user_resistors
                        clear user calibration resistors
  -p [POWER_CYCLE_DUT], --power_cycle_dut [POWER_CYCLE_DUT]
                        power cycle the DUT and delay
  -v, --verbose         print logging information
  -o OUT_FILE, --out_file OUT_FILE
                        write measurement data to file
  -g, --spike_filtering
                        enable spike filtering
  -x, --enable_ext_trigger
                        enable 'TRIG IN' external trigger
  -k, --skip_verify     save time by not verifying the PPK firmware
  -f, --force           program the PPK firmware if necessary
```

