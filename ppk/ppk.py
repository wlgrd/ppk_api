"""
Handles communication with Nordic Power Profiler Kit (PPK) hardware
using Segger's Real-Time Transfer functionality.
"""
import time
import struct
import re

from numpy import average as np_avg

from pynrfjprog import APIError


class PPKError(Exception):
    """PPK exception class, inherits from the built-in Exception class."""

    def __init__(self, error=None):
        """Constructs a new object and sets the error."""
        self.error = error
        err_str = 'PPK error: {}'.format(self.error)
        Exception.__init__(self, err_str)


class RTTCommand():
    """RTT command opcodes."""
    TRIGGER_SET = 0x01
    TRIG_WINDOW_SET = 0x03
    TRIG_SINGLE_SET = 0x05
    AVERAGE_START = 0x06
    AVERAGE_STOP = 0x07
    TRIG_STOP = 0x0A
    DUT_TOGGLE = 0x0C
    REGULATOR_SET = 0x0D
    VREF_LO_SET = 0x0E
    VREF_HI_SET = 0x0F
    RES_USER_SET = 0x12
    RES_USER_CLEAR = 0x13
    SPIKE_FILTER_ON = 0x15
    SPIKE_FILTER_OFF = 0x16


class API():
    """The PPK API takes care of rtt connection to the ppk, reading average
    current consumption, starting and stopping etc.
    """
    ADC_SAMPLING_TIME_US = 13
    SAMPLES_PER_AVERAGE = 10
    AVERAGE_TIME_US = (SAMPLES_PER_AVERAGE * ADC_SAMPLING_TIME_US)
    TRIGGER_SAMPLES_PER_SECOND = (1e6 / ADC_SAMPLING_TIME_US)
    AVERAGE_SAMPLES_PER_SECOND = (1e6 / AVERAGE_TIME_US)

    EXT_REG_MIN_MV = 2100
    EXT_REG_MAX_MV = 3600

    TRIG_WINDOW_MIN_US = 5850
    TRIG_WINDOW_MAX_US = 52000

    RTT_CHANNEL_INDEX = 0
    RTT_READ_BUF_LEN = 500

    PPK_CMD_WRITE_DELAY = 0.25

    def __init__(self, nrfjprog_api, logprint=True):
        """A stateful interface to a Nordic Power Profiler Kit."""
        self.nrfjprog_api = nrfjprog_api
        self.logprint = logprint
        self._connected = False
        self._vdd = None
        self._metadata = None
        self._resistors = None

    def connect(self):
        """Connect to the PPK and gather metadata."""
        self.nrfjprog_api.sys_reset()
        self.nrfjprog_api.go()
        self.nrfjprog_api.rtt_start()
        while not self.nrfjprog_api.rtt_is_control_block_found():
            continue
        # Allow the PPK firmware to start.
        time.sleep(0.9)
        metadata = self.nrfjprog_api.rtt_read(self.RTT_CHANNEL_INDEX, self.RTT_READ_BUF_LEN)
        self._metadata = self._parse_metadata(metadata)
        self._log(self._metadata)
        self._vdd = int(self._metadata["VDD"])
        if self._metadata['USER_R1']:
            self._resistors = [float(self._metadata['USER_R1']),
                               float(self._metadata['USER_R2']),
                               float(self._metadata['USER_R3'])]
        else:
            self._resistors = [float(self._metadata['R1']),
                               float(self._metadata['R2']),
                               float(self._metadata['R3'])]
        self._connected = True

    def reset_connection(self):
        """Stop RTT, flush it, and then connect to the PPK again."""
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self.nrfjprog_api.rtt_stop()
        self._flush_rtt()
        self.connect()

    def get_metadata(self):
        """Return a copy of the PPK metadata that is read at the start of the connection."""
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        return self._metadata.copy()

    def enable_dut_power(self):
        """Turn DUT power on."""
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("DUT power on.")
        self._write_ppk_cmd([RTTCommand.DUT_TOGGLE, 1])

    def disable_dut_power(self):
        """Turn DUT power off."""
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("DUT power off.")
        self._write_ppk_cmd([RTTCommand.DUT_TOGGLE, 0])

    def clear_user_resistors(self):
        """Clear user resistors."""
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("Clearing user resistors.")
        self._resistors = [float(self._metadata['R1']),
                           float(self._metadata['R2']),
                           float(self._metadata['R3'])]
        self._write_ppk_cmd([RTTCommand.RES_USER_CLEAR])

    def set_user_resistors(self, user_r1, user_r2, user_r3):
        """Set USER_R1, USER_R2, and USER_R3 resistors. Values should
        be floats and will be packed into four bytes.
        """
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("Set user resistors: %.3f, %.3f, %.3f." % (user_r1, user_r2, user_r3))
        self._resistors = [user_r1, user_r2, user_r3]
        cmd = [RTTCommand.RES_USER_SET]
        cmd.extend(struct.pack('f', user_r1))
        cmd.extend(struct.pack('f', user_r2))
        cmd.extend(struct.pack('f', user_r3))
        self._write_ppk_cmd(cmd)

    def enable_spike_filtering(self):
        """Enable spike filtering feature."""
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("Enabling spike filtering.")
        self._write_ppk_cmd([RTTCommand.SPIKE_FILTER_ON])

    def disable_spike_filtering(self):
        """Disable spike filtering feature."""
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("Disabling spike filtering.")
        self._write_ppk_cmd([RTTCommand.SPIKE_FILTER_OFF])

    def set_external_reg_vdd(self, vdd):
        """Set VDD of external voltage regulator.

        This is only recommended when using an external DUT (i.e. not the DK with a PPK
        sitting on top of it). The DUT should be powered via the 'External DUT' pins.
        The 'DUT Select' switch should be set to 'External'. The 'Power Select'
        switch should be set to 'Reg.'.
        """
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("Setting external regulator VDD to %d." % vdd)
        # Setting voltages above or below these values can cause the emu connection to stall.
        if (self.EXT_REG_MIN_MV > vdd) or (self.EXT_REG_MAX_MV < vdd):
            raise PPKError("Invalid vdd given to set_external_reg_vdd: (%d)." % vdd)
        target_vdd = vdd
        while True:
            if target_vdd > self._vdd:
                next_vdd = self._vdd + 100 if abs(target_vdd - self._vdd) > 100 else target_vdd
            else:
                next_vdd = self._vdd - 100 if abs(target_vdd - self._vdd) > 100 else target_vdd
            vdd_high_byte = next_vdd >> 8
            vdd_low_byte = next_vdd & 0xFF
            self._write_ppk_cmd([RTTCommand.REGULATOR_SET, vdd_high_byte, vdd_low_byte])
            self._vdd = next_vdd
            # A short delay between calls to _write_ppk_cmd improves stability.
            if self._vdd == target_vdd:
                break
            else:
                time.sleep(self.PPK_CMD_WRITE_DELAY)

    def set_trigger_window(self, time_us):
        """Set the trigger window. This is the dataset transferred
        upon every trigger from the PPK.
        """
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("Set trigger window to %dus." % time_us)
        if (self.TRIG_WINDOW_MIN_US > time_us) or (self.TRIG_WINDOW_MAX_US < time_us):
            raise PPKError("Invalid time_us given to set_trigger_window: (%d)." % time_us)
        window = int(time_us / self.ADC_SAMPLING_TIME_US)
        high = (window >> 8) & 0xFF
        low = window & 0xFF
        self._write_ppk_cmd([RTTCommand.TRIG_WINDOW_SET, high, low])

    def measure_average(self, time_s, discard_jitter_count=500):
        """Collect time_s worth of average measurements and return
        the average along with the list of measurements.
        """
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("measure_average(%d, %d)." % (time_s, discard_jitter_count))
        samples_count = (time_s * self.AVERAGE_SAMPLES_PER_SECOND)
        ppk_helper = PPKDataHelper()
        self._start_average_measurement()
        while True:
            self._read_and_parse_ppk_data(ppk_helper)
            collected_buffs = len(ppk_helper)
            self._log("Collecting samples: %d" % collected_buffs, end='\r')
            if samples_count <= collected_buffs:
                break
        self._log('')
        self._stop_average_measurement()
        self._flush_rtt()
        avg_buf = ppk_helper.get_averages()
        avg_buf = avg_buf[discard_jitter_count:]
        return (np_avg(avg_buf), avg_buf)

    def measure_triggers(self, window_time_us, level_ua, count=1):
        """Collect count trigger buffers."""
        if not self._connected:
            raise PPKError("Invalid operation: must connect first.")
        self._log("measure_triggers(%r, %r, %r)." % (window_time_us, level_ua, count))
        ppk_helper = PPKDataHelper()
        self.set_trigger_window(window_time_us)
        if count == 1:
            self._set_single_trigger(level_ua)
        else:
            self._set_trigger(level_ua)
        # Each buffer includes a timestamp.
        samples_count = count * 2
        while True:
            self._read_and_parse_ppk_data(ppk_helper)
            collected_buffs = len(ppk_helper)
            self._log("Collecting trigger buffers: %d" % (collected_buffs/2), end='\r')
            if samples_count <= collected_buffs:
                break
        self._log('')
        self._stop_trigger()
        self._flush_rtt()
        result = []
        for timestamp, buf in ppk_helper.get_trigger_buffs(*self._resistors):
            result.append((timestamp, np_avg(buf), buf))
        return result

    def _start_average_measurement(self):
        """Start generating average current measurements."""
        self._log("Starting average measurement.")
        self._write_ppk_cmd([RTTCommand.AVERAGE_START])

    def _stop_average_measurement(self):
        """Stop generating average current measurements."""
        self._log("Stopping average measurement.")
        self._write_ppk_cmd([RTTCommand.AVERAGE_STOP])

    def _set_trigger(self, level_ua):
        """Set the trigger level.

        The level_ua parameter is the current draw (in microamps) that
        will activate the trigger. To convert from milliamps simply
        multiply by 1000.
        """
        self._log("Set trigger to %duA." % level_ua)
        high = (level_ua >> 16) & 0xFF
        mid = (level_ua >> 8) & 0xFF
        low = level_ua & 0xFF
        self._write_ppk_cmd([RTTCommand.TRIGGER_SET, high, mid, low])

    def _set_single_trigger(self, level_ua):
        """Set the single trigger level.

        The level_ua parameter is the current draw (in microamps) that
        will activate the trigger. To convert from milliamps simply
        multiply by 1000.
        """
        self._log("Set single trigger to %duA." % level_ua)
        high = (level_ua >> 16) & 0xFF
        mid = (level_ua >> 8) & 0xFF
        low = level_ua & 0xFF
        self._write_ppk_cmd([RTTCommand.TRIG_SINGLE_SET, high, mid, low])

    def _stop_trigger(self):
        """Disable trigger buffer generation."""
        self._log("Stopping trigger.")
        self._write_ppk_cmd([RTTCommand.TRIG_STOP])

    def _write_ppk_cmd(self, byte_array):
        """Adds escape characters to byte_array and then writes it to RTT."""
        self.nrfjprog_api.rtt_write(self.RTT_CHANNEL_INDEX,
                                    PPKDataHelper.encode(byte_array),
                                    encoding=None)

    def _read_and_parse_ppk_data(self, ppk_data_helper):
        """Read bytes from the RTT channel and pass them to a PPKDataHelper.

        Read [zero, RTT_READ_BUF_LEN] bytes from the RTT channel and use the
        helper to decode them.
        """
        byte_array = self.nrfjprog_api.rtt_read(self.RTT_CHANNEL_INDEX,
                                                self.RTT_READ_BUF_LEN,
                                                encoding=None)
        for byte in byte_array:
            ppk_data_helper.decode(byte)

    def _flush_rtt(self):
        while True:
            flush_bytes = self.nrfjprog_api.rtt_read(self.RTT_CHANNEL_INDEX,
                                                     self.RTT_READ_BUF_LEN,
                                                     encoding=None)
            if not flush_bytes:
                break

    def _log(self, logstring, **kwargs):
        """Print lof information only when logprint was set to True in __ini__."""
        if self.logprint:
            print(logstring, **kwargs)

    @staticmethod
    def _parse_metadata(metadata_str):
        """Use a Regular Expression to parse a metadata packet."""
        metadata_fields = ("VERSION", "CAL", "R1", "R2", "R3", "BOARD_ID",
                           "USER_R1", "USER_R2", "USER_R3", "VDD", "HI", "LO")
        re_string = ('').join([
            'VERSION\\s*([^\\s]+)\\s*CAL:\\s*(\\d+)\\s*',
            '(?:R1:\\s*([\\d.]+)\\s*R2:\\s*([\\d.]+)\\s*R3:\\s*',
            '([\\d.]+))?\\s*Board ID\\s*([0-9A-F]+)\\s*',
            '(?:USER SET\\s*R1:\\s*([\\d.]+)\\s*R2:\\s*',
            '([\\d.]+)\\s*R3:\\s*([\\d.]+))?\\s*',
            'Refs\\s*VDD:\\s*(\\d+)\\s*HI:\\s*(\\d.+)\\s*LO:\\s*(\\d+)',
        ])
        result = re.split(re_string, metadata_str)[1:]
        metadata = {metadata_fields[i]:result[i] for i in range(0, len(metadata_fields))}
        return metadata


class PPKDataHelper():
    """Encodes and decodes PPK byte arrays.

    Encodes byte arrays via a class function. Decoding is a stateful
    operation so an object must be instantiated. Decoded packets can
    be retrieved as a list or iterated over.
    """
    STX_BYTE = 0x02
    ETX_BYTE = 0x03
    ESC_BYTE = 0x1F

    MODE_RECV = 1
    MODE_ESC_RECV = 2

    AVERAGE_PKT_LEN = 4
    TIMESTAMP_PKT_LEN = 5

    # Trigger buffer values consist of pairs of two bytes that get packed
    # into a uint16 with the top three bits encoding the "measurement range".
    # The measurement range is defined by the R1, R2, R3 resistors or
    # their USER replacements.
    MEAS_RANGE_POS = 14
    MEAS_RANGE_MSK = (3 << 14)
    MEAS_RANGE_LO = 1
    MEAS_RANGE_MID = 2
    MEAS_RANGE_HI = 3

    MEAS_ADC_POS = 0
    MEAS_ADC_MSK = 0x3FFF

    ADC_REF = 0.6
    ADC_GAIN = 4.0
    ADC_MAX = 8192.0
    ADC_MULT = (ADC_REF / (ADC_GAIN * ADC_MAX))

    def __init__(self):
        """Creates an empty object for parsing bytes."""
        self._read_mode = self.MODE_RECV
        self._buf = []
        self._decoded = []

    def __iter__(self):
        return self

    def __next__(self):
        if not self._decoded:
            raise StopIteration
        else:
            return self._decoded.pop(0)

    def __len__(self):
        return len(self._decoded)

    def decode(self, byte):
        """Decode a single byte from the PPK.

        Return True if the byte completes the decoding of a packet.
        """
        if self.MODE_RECV == self._read_mode:
            if self.ESC_BYTE == byte:
                self._read_mode = self.MODE_ESC_RECV
            elif self.ETX_BYTE == byte:
                self._decoded.append(self._buf.copy())
                self._buf.clear()
                self._read_mode = self.MODE_RECV
                return True
            else:
                self._buf.append(byte)
        elif self.MODE_ESC_RECV == self._read_mode:
            self._buf.append(byte ^ 0x20)
            self._read_mode = self.MODE_RECV
        return False

    def get_decoded(self):
        """Return the list of decoded packets."""
        return self._decoded

    def get_averages(self):
        """Returns a list of unpacked average packets."""
        return [self.unpack_average(p) for p in self._decoded if self.AVERAGE_PKT_LEN == len(p)]

    def get_trigger_buffs(self, meas_res_lo, meas_res_mid, meas_res_hi):
        timestamps = [self.unpack_timestamp(x) for x in self._decoded[::2]]
        u16s = []
        # Condense trigger buffer values into uint16s.
        for pkt in self._decoded[1::2]:
            u16s.append([self.make_u16(pkt[x], pkt[x+1]) for x in range(0, len(pkt), 2)])
        # Decode the measurement range for the buffer values.
        meas_range = (u16s[0][0] & self.MEAS_RANGE_MSK) >> self.MEAS_RANGE_POS
        # Scale the buffer values accordingly.
        divisor = None
        if meas_range == self.MEAS_RANGE_LO:
            divisor = meas_res_lo
        elif meas_range == self.MEAS_RANGE_MID:
            divisor = meas_res_mid
        elif meas_range == self.MEAS_RANGE_HI:
            divisor = meas_res_hi
        else:
            raise ValueError("Invalid measurement range in trigger buffer: %d" % meas_range)
        buffs = []
        for buf in u16s:
            buffs.append([x * (self.ADC_MULT / divisor) for x in buf])
        return zip(timestamps, buffs)

    def reset(self):
        """Clear the state of the object."""
        self._read_mode = self.MODE_RECV
        self._buf = []
        self._decoded = []

    @classmethod
    def encode(cls, byte_array):
        """Return a byte array with added PPK escape characters."""
        buf = []
        buf.append(cls.STX_BYTE)
        for byte in byte_array:
            if byte in (cls.STX_BYTE, cls.ETX_BYTE, cls.ESC_BYTE):
                buf.append(cls.ESC_BYTE)
                buf.append(byte ^ 0x20)
            else:
                buf.append(byte)
        buf.append(cls.ETX_BYTE)
        return buf

    @classmethod
    def is_average_pkt(cls, byte_array):
        """Return True if byte_array appears to contain average data."""
        return cls.AVERAGE_PKT_LEN == len(byte_array)

    @classmethod
    def is_timestamp_pkt(cls, byte_array):
        """Return True if byte_array appears to contain timestamp data."""
        return cls.TIMESTAMP_PKT_LEN == len(byte_array)

    @classmethod
    def is_trigger_pkt(cls, byte_array):
        """Return True if byte_array appears to contain trigger data."""
        return not cls.is_average_pkt(byte_array) and not cls.is_timestamp_pkt(byte_array)

    @staticmethod
    def unpack_average(byte_array):
        """Decode the four bytes in byte_array into a float."""
        return struct.unpack('<f', bytearray(byte_array))[0]

    @staticmethod
    def unpack_timestamp(byte_array):
        """Decode the first four bytes in byte_array and return a u32."""
        return struct.unpack('<I', bytearray(byte_array[:4]))[0]

    @staticmethod
    def make_u16(low_byte, high_byte):
        return (high_byte << 8) + low_byte
