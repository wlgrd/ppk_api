Nordic Semiconductor's [Power Profiler Kit (PPK)](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/Power-Profiler-Kit) is very useful for measuring and optimizing power consumption when developing battery-powered devices. Although the [nRF Connect for Desktop](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Connect-for-desktop) application provides a friendly GUI, it has limited support for logging measurement data and isn't accessible from a command line. The purpose of this library is to make the PPK more useful for logging and integration into automated testing/continuous integration systems.

### Features
The main features of the **ppk_api** include:

 - Comparable functionality to the nRF Connect for Desktop GUI
 - Comma Separated Value (CSV) log files for both average and trigger buffer measurement data
 - Written in Python3 for easy integration with other CLI tools
 - Cross-platform support

### Requirements
The interface to the PPK requires Nordic's [nRF Command Line Tools](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Command-Line-Tools).

Additionally, the excellent [nRF Pynrfjprog](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Pynrfjprog) and [pandas](https://pandas.pydata.org/) Python modules can be installed from the command line using pip:
```
$ cd ppk_api
$ pip3 install --user -r requirements.txt
```

### Usage
**main.py** provides an example of how to connect to a PPK and use the API directly. Otherwise, help is available from the command line:

```
$ python3 main.py -h
usage: main.py [-h] [-s SERIAL_NUMBER] [-a AVERAGE]
               [-w [TRIGGER_MICROSECONDS]] [-n [TRIGGER_COUNT]]
               [-e EXTERNAL_VDD] [-c] [-p [POWER_CYCLE_DUT]] [-v]
               [-o OUT_FILE] [-z] [-g] [-t TRIGGER_MICROAMPS | -x] [-k | -f]

optional arguments:
  -h, --help            show this help message and exit
  -s SERIAL_NUMBER, --serial_number SERIAL_NUMBER
                        serial number of J-Link
  -a AVERAGE, --average AVERAGE
                        print average current over time
  -w [TRIGGER_MICROSECONDS], --trigger_microseconds [TRIGGER_MICROSECONDS]
                        set trigger window in microseconds [1000, 52000]
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
  -z, --png             create .png graph(s) of data in out_file
  -g, --spike_filtering
                        enable spike filtering
  -t TRIGGER_MICROAMPS, --trigger_microamps TRIGGER_MICROAMPS
                        set trigger threshold in microamps
  -x, --enable_ext_trigger
                        enable 'TRIG IN' external trigger
  -k, --skip_verify     save time by not verifying the PPK firmware
  -f, --force           program the PPK firmware if necessary
```
By default, the CLI will verify that the PPK has been programmed with the included PPK firmware every time it starts. If the firmware is not found then an error message is generated and the program exits. The **--force** option can be used to automatically reprogram the PPK without generating an error. The firmware verification can be skipped entirely using the **--skip_verify** option.

If only one J-Link/Nordic development kit is plugged into the PC then the **--serial_number** option can be skipped.

**EXAMPLE:** Reset the DUT and allow 3 seconds to press 'Button4' before collecting 5 seconds of average measurement data':
```
$ python3 main.py -p 3 -a 5
Average: 865.98uA
```
Adding **--verbose** prints additional information:
```
 $ python3 main.py -p 3 -a 5 -v
{'R3': '1.845', 'USER_R3': None, 'R1': '510.000', 'USER_R2': None, 'HI': '20000 ', 'LO': '44270', 'VDD': '3000', 'CAL': '0', 'R2': '30.500', 'BOARD_ID': 'BB49651B', 'USER_R1': None, 'VERSION': 'ppk-fw-2.0.0'}
Resistors: LO: 510.0, MID: 30.5, HI: 1.845
DUT power off.
DUT power on.
Sleep for 3.0s.
measure_average(5, 500).
Starting average measurement.
Collecting samples: 38516
Stopping average measurement.
Average: 858.66uA
```

**EXAMPLE:** Reset the DUT and allow one second for the firmware to boot before collecting a trigger buffer with a 2mA "Trigger level" and 5.85ms of sample data. Save the data to a file called 'trig_data.csv':
```
$ python3 main.py -p 1 -t 2000 -w 5850 -o trig_data.csv
Trigger buff average: 2649.05uA
```
The 'trig_data.csv' file then looks like this:
```
Timestamp (us),Current (uA)
139129,544.5136398565573
139142,2201.468045594262
139155,2197.265625
139168,2185.2587090163934
...
```
And LibreOffice can draw a chart of the data in 'trig_data.csv' as a sanity check: 
![trigger_buf](https://user-images.githubusercontent.com/6494431/61754172-f2020980-ad66-11e9-913d-870641a9f7f4.png)

**NOTE:** The **--external_vdd** option is only recommended for use when the DUT is an external device that is being powered by 'External DUT' pins. See the [PPK documentation](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fug_ppk%2FUG%2Fppk%2FPPK_user_guide_Intro.html&cp=6_6&tags=Power+Profiler+Kit) for more information.

**NOTE:** When a connection to the PPK is established a soft reset is performed on the PPK to put its firmware into a known state. The side effect of this action is that the DUT experiences a power cycle. **This can lead to confusion if the DUT needs a certain amount of time to boot before it's ready to be measured or a stateful action needs to be performed (e.g. pushing a button on the DUT to enter a mode).** If this is the case then the **--power_cycle_dut** option can be used to provide a deterministic delay (in seconds) between the DUT being reset and the start of the measurement.
