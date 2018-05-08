# test regex
# regex lib

import re
from ppk.ppk import API as ppkapi
from pynrfjprog import API

# rtt = API.API('NRF52')p
# rtt.open()
# rtt.connect_to_emu_without_snr()

# ppk = ppkapi(rtt, logprint=True)
# ppk.connect()

ppkstring = 'VERSION ppk-fw-2.0.0CAL:1 R1:466.906 R2:30.325 R3:14.996 Board ID 5D6C9120 \n\nUSER SET R1:466.906 R2:30.325 R3:14.996\nRefs VDD: 2906 HI: 20000 LO: 44270'
restring = ('').join([
    'VERSION\\s*([^\\s]+)\\s*CAL:\\s*(\\d+)\\s*',
    '(?:R1:\\s*([\\d.]+)\\s*R2:\\s*([\\d.]+)\\s*R3:\\s*([\\d.]+))?\\s*Board ID\\s*([0-9A-F]+)\\s*',
    '(?:USER SET\\s*R1:\\s*([\\d.]+)\\s*R2:\\s*([\\d.]+)\\s*R3:\\s*([\\d.]+))?\\s*',
    'Refs\\s*VDD:\\s*(\\d+)\\s*HI:\\s*(\\d.+)\\s*LO:\\s*(\\d+)',
])
result = (re.split(restring, ppkstring))
result = result[1:13]

version, calibrationDone, resLow, resMid, resHi, boardID, userResLow, userResMid, userRedHi, vdd, vrefHigh, vrefLow = result

print(result)
print (boardID)

