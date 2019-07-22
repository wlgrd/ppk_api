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
    # NOTE: Not implemented as an Enum because these values
    #       need to be treated as bytes.
    TRIGGER_SET = 0x01 # following trigger of type int16
    AVG_NUM_SET = 0x02 # Number of samples x16 to average over
    TRIG_WINDOW_SET = 0x03 # following window of type unt16
    TRIG_INTERVAL_SET = 0x04 #
    TRIG_SINGLE_SET = 0x05
    AVERAGE_START = 0x06
    AVERAGE_STOP = 0x07
    RANGE_SET = 0x08
    LCD_SET = 0x09
    TRIG_STOP = 0x0A
    CALIBRATE_OFFSET = 0x0B
    DUT_TOGGLE = 0x0C
    REGULATOR_SET = 0x0D
    VREF_LO_SET = 0x0E
    VREF_HI_SET = 0x0F
    RES_SET = 0x10
    EXT_TRIG_TOGGLE = 0x11
    RES_USER_SET = 0x12
    RES_USER_CLEAR = 0x13
    RES_CAL_CLEAR = 0x14
    SPIKE_FILTER_ON = 0x15
    SPIKE_FILTER_OFF = 0x16


class API():
    """
        The PPK API takes care of rtt connection to the ppk, reading average
        current consumption, starting and stopping etc.

        @param int nrfjprog_api : Instance to a pynrfjprog API object
        @param logprint         : Turn on/off logging for the module
    """
    DATA_TYPE_TRIGGER = 0
    DATA_TYPE_AVERAGE = 1

    ADC_SAMPLING_TIME_US = 13
    SAMPLES_PER_AVERAGE = 10
    AVERAGE_TIME_US = (SAMPLES_PER_AVERAGE * ADC_SAMPLING_TIME_US)
    TRIGGER_SAMPLES_PER_SECOND = (1e6 / ADC_SAMPLING_TIME_US)
    AVERAGE_SAMPLES_PER_SECOND = (1e6 / AVERAGE_TIME_US)

    SAMPLE_INTERVAL_US = 13e-6
    SAMPLE_INTERVAL_MS = 13e-3

    ADC_REF = 0.6
    ADC_GAIN = 4.0
    ADC_MAX = 8192.0

    MEAS_RES_HI = 1.8
    MEAS_RES_MID = 28.0
    MEAS_RES_LO = 500.0

    MEAS_RANGE_NONE = 0
    MEAS_RANGE_LO = 1
    MEAS_RANGE_MID = 2
    MEAS_RANGE_HI = 3
    MEAS_RANGE_INVALID = 4

    MEAS_RANGE_POS = 14
    MEAS_RANGE_MSK = (3 << 14)

    MEAS_ADC_POS = 0
    MEAS_ADC_MSK = 0x3FFF

    VDD_SET_MIN = 2100
    VDD_SET_MAX = 3600

    RTT_CHANNEL_INDEX = 0
    RTT_READ_BUF_LEN = 200

    PPK_CMD_WRITE_DELAY = 0.25

    def __init__(self, nrfjprog_api, logprint=True):
        """A stateful interface to a Nordic Power Profiler Kit."""
        self.nrfjprog_api = nrfjprog_api
        self.logprint = logprint        # Connect to instrument
        self.current_meas_range = 0  # Dummy setting
        self.trigger_buffer = []
        self.trigger_buffer_size = 0
        self.trigger_data_captured = 0
        self._connected = False
        self._vdd = None
        self._metadata = {}

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
        self._vdd = int(self._metadata["VDD"])
        self._connected = True

    def reset_connection(self):
        """Stop RTT, flush it, and then connect to the PPK again."""
        self.nrfjprog_api.rtt_stop()
        self._flush_rtt()
        self.connect()

    def get_metadata(self):
        """Return a copy of the PPK metadata that is read at the start of the connection."""
        return self._metadata.copy()

    def dut_power_on(self):
        self._log("DUT power on.")
        self._write_ppk_cmd([RTTCommand.DUT_TOGGLE, 1])

    def dut_power_off(self):
        self._log("DUT power off.")
        self._write_ppk_cmd([RTTCommand.DUT_TOGGLE, 0])

    def trigger_acquisition_time_set(self, acqtime):
        """ Set the acquisition window in ms
            This is the dataset transferred upon every trigger
            from the PPK.
        """
        self.trigger_buffer_size = int(acqtime/self.SAMPLE_INTERVAL_MS + 1)
        buffer_size_high = (self.trigger_buffer_size >> 8) & 0xFF
        buffer_size_low = self.trigger_buffer_size & 0xFF
        self._write_ppk_cmd([RTTCommand.TRIG_WINDOW_SET,
                             buffer_size_high, buffer_size_low])
        self._log("Set acqusition time %d (buffer size:%d)." %
                  (acqtime, self.trigger_buffer_size))

    def trigger_value_set(self, trigger):
        """ Set the trigger value in uA
            The trigger will send data if this value is reached
        """
        high = (trigger >> 16) & 0xFF
        mid = (trigger >> 8) & 0xFF
        low = trigger & 0xFF
        self._write_ppk_cmd([RTTCommand.TRIGGER_SET, high, mid, low])
        self._log("Trigger set to %d." % trigger)

    def trigger_stop(self):
        self._write_ppk_cmd([RTTCommand.TRIG_STOP])

    def measure_average(self, time_s, discard_jitter_count=500):
        """Collect time_s worth of average measurements and return a float."""
        samples_count = (time_s * self.AVERAGE_SAMPLES_PER_SECOND)
        ppk_helper = PPKDataHelper()
        self.start_average_measurement()
        while True:
            self._read_and_parse_ppk_data(ppk_helper)
            if samples_count <= len(ppk_helper):
                break
            self._log("Collecting samples: %d" % len(ppk_helper), end='\r')
        self._log('')
        self.stop_average_measurement()
        self._flush_rtt()
        avg_buf = ppk_helper.get_averages()
        avg_buf = avg_buf[discard_jitter_count:]
        return np_avg(avg_buf)

    def trigger_single(self):
        """"""
        pass
        #self._log("Trigger data received of len %d." % len(byte_array))
        #     for i in range(0, len(byte_array), 2):
        #         if (i + 1) < len(byte_array):
        #             tmp = np.uint16((byte_array[i + 1] << 8) + byte_array[i])
        #             self.current_meas_range = (tmp & MEAS_RANGE_MSK) >> MEAS_RANGE_POS
        #             adc_val = (tmp & MEAS_ADC_MSK) >> MEAS_ADC_POS
        #             self.trigger_data_handler(adc_val, self.current_meas_range)

    def start_average_measurement(self):
        self._log("Starting average measurement.")
        self._write_ppk_cmd([RTTCommand.AVERAGE_START])

    def stop_average_measurement(self):
        self._log("Stopping average measurement.")
        self._write_ppk_cmd([RTTCommand.AVERAGE_STOP])

    def set_external_reg_vdd(self, vdd):
        """Set VDD of external voltage regulator."""
        self._log("Setting external regulator VDD to %d." % vdd)
        # Setting voltages above or below these values can cause the emu connection to stall.
        if (self.VDD_SET_MIN > vdd) or (self.VDD_SET_MAX < vdd):
            raise PPKError("Invalid vdd given to set_external_reg_vdd (%d)." % vdd)
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

    def clear_user_resistors(self):
        self._write_ppk_cmd([RTTCommand.RES_USER_CLEAR])

    def clear_cal_resistors(self):
        self._log("Clearing calibration resistors.")
        self._write_ppk_cmd([RTTCommand.RES_CAL_CLEAR])

    def write_new_resistors(self, resistors):
        """
        resistors[0] = r1 = low measurements (~510)
        resistors[1] = r2 = mid measurements (~31)
        resistors[2] = r3 = high measurements (~1.8)
        """
        r1_list = []
        r2_list = []
        r3_list = []

        # if (resistors[1] >=32.5 and resistors[1] < 33):
        #     self._log("Changing resistor value")
        #     resistors[1] = 32.4

        self._log("writing %.3f, %.3f, %.3f" %(resistors[0],
                                               resistors[1],
                                               resistors[2]))
        # Pack the floats
        bufr1 = struct.pack('f', resistors[0])
        bufr2 = struct.pack('f', resistors[1])
        bufr3 = struct.pack('f', resistors[2])
        # PPK receives byte packages, put them in a list
        for byte in bufr1:
            r1_list.append(byte)
        for byte in bufr2:
            r2_list.append(byte)
        for byte in bufr3:
            r3_list.append(byte)

        # Write the floats to PPK
        self._write_ppk_cmd([RTTCommand.RES_SET,
                             (r1_list[0]), (r1_list[1]), (r1_list[2]), (r1_list[3]),
                             (r2_list[0]), (r2_list[1]), (r2_list[2]), (r2_list[3]),
                             (r3_list[0]), (r3_list[1]), (r3_list[2]), (r3_list[3])
                            ])

    def trigger_data_handler(self):
        adc_val = 0

        if self.trigger_data_captured >= self.trigger_buffer_size:
            self._log("Trigger buffer full.")
            self.trigger_stop()
            # TODO: What is this?
            # self.data_callback(self.trigger_buffer, self.DATA_TYPE_TRIGGER)
            self.trigger_data_captured = 0
            self.trigger_buffer = []
        else:
            self.trigger_data_captured += 1
            if self.current_meas_range == self.MEAS_RANGE_LO:
                lo_div = (self.ADC_GAIN * self.ADC_MAX * self.MEAS_RES_LO)
                sample_ua = adc_val * (self.ADC_REF / lo_div)
            elif self.current_meas_range == self.MEAS_RANGE_MID:
                mid_div = (self.ADC_GAIN * self.ADC_MAX * self.MEAS_RES_MID)
                sample_ua = adc_val * (self.ADC_REF / mid_div)
            elif self.current_meas_range == self.MEAS_RANGE_HI:
                hi_div = (self.ADC_GAIN * self.ADC_MAX * self.MEAS_RES_HI)
                sample_ua = adc_val * (self.ADC_REF / hi_div)
            elif self.current_meas_range == self.MEAS_RANGE_INVALID:
                self._log("Range INVALID")
            elif self.current_meas_range == self.MEAS_RANGE_NONE:
                self._log("Range not detected")
            self.trigger_buffer.append(sample_ua)

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
    def unpack_average(cls, byte_array):
        """Decode the four bytes in byte_array into a float."""
        return struct.unpack('<f', bytearray(byte_array))[0]

    @classmethod
    def unpack_timestamp(cls, byte_array):
        """Decode the first four bytes in byte_array and return a u32."""
        return struct.unpack('<I', bytearray(byte_array[:4]))[0]

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
