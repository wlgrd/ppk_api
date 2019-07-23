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
cd ppk_api
pip3 install --user -r requirements.txt
```

