"""Handles communication with PPK hardware using RTT."""
import time
import struct
import re
import math
import threading


SAMPLE_INTERVAL_US = 13e-6
SAMPLE_INTERVAL_MS = 13e-3

SAMPLE_INTERVAL = 13.0e-6
ADC_REF = 0.6
ADC_GAIN = 4.0
ADC_MAX = 8192.0

MEAS_RES_HI = 1.8
MEAS_RES_MID = 28.0
MEAS_RES_LO = 500.0

STX = 0x02
ETX = 0x03
ESC = 0x1F

MODE_IDLE = 0
MODE_RECV = 1
MODE_ESC_RECV = 2

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
    SINGLE_TRIG = 0x05
    RUN = 0x06
    STOP = 0x07
    RANGE_SET = 0x08
    LCD_SET = 0x09
    TRIG_STOP = 0x0A
    CALIBRATE_OFFSET = 0x0B
    DUT = 0x0C
    SETVDD = 0x0D
    SETVREFLO = 0x0E
    SETVREFHI = 0x0F
    SET_RES = 0x10
    CLEAR_RES_USER = 0x13
    CLEAR_RES_CAL = 0x14


class API():
    """
        The PPK API takes care of rtt connection to the ppk, reading average
        current consumption, starting and stopping etc.

        @param int rtt_instance : Instance to a pynrfjprog API object
        @param logprint         : Turn on/off logging for the module
    """
    DATA_TYPE_TRIGGER = 0
    DATA_TYPE_AVERAGE = 1

    def __init__(self, rtt_instance, logprint=True):
        """"""
        self.nrfjprog = rtt_instance
        self.alive = False
        self.logprint = logprint        # Connect to instrument
        self.current_meas_range = 0  # Dummy setting
        self.board_id = None
        self.trigger_buffer = []
        self.trigger_buffer_size = 0
        self.trigger_data_captured = 0
        self.avg_buffer = []
        self.avg_timeout = 0
        self.finished = False
        self.m_vdd = 3000
        self.connected = False
        self.read_mode = MODE_RECV
        self.data_buffer = []
        self.read_thread = None
        self.calibration_data = None
        self.timeout = None

    def _get_metadata(self, rttdata):
        self.log("Metadata: " + rttdata)
        restring = ('').join([
            'VERSION\\s*([^\\s]+)\\s*CAL:\\s*(\\d+)\\s*',
            '(?:R1:\\s*([\\d.]+)\\s*R2:\\s*([\\d.]+)\\s*R3:\\s*',
            '([\\d.]+))?\\s*Board ID\\s*([0-9A-F]+)\\s*',
            '(?:USER SET\\s*R1:\\s*([\\d.]+)\\s*R2:\\s*',
            '([\\d.]+)\\s*R3:\\s*([\\d.]+))?\\s*',
            'Refs\\s*VDD:\\s*(\\d+)\\s*HI:\\s*(\\d.+)\\s*LO:\\s*(\\d+)',
        ])

        # Parse the given data using the regex restring
        result = re.split(restring, rttdata)
        return result[1:13]

    def log(self, logstring):
        """Adds a prefix for log string prints."""
        if self.logprint:
            print("PPK: %s" % str(logstring))

    def get_connected_board_id(self):
        """Returns current board_id or raises a PPKError."""
        if self.board_id is None:
            raise PPKError("Board ID not read at connect")
        return self.board_id

    def disconnect_api(self):
        """Disconnect from the emulator. TODO: Is this necessary?"""
        pass
        #self.nrfjprog.disconnect()

    def connect(self):
        try:
            self.nrfjprog.sys_reset()
        except AttributeError:
            pass
        self.nrfjprog.go()
        self.nrfjprog.rtt_start()
        while not self.nrfjprog.rtt_is_control_block_found():
            continue
        try:
            time.sleep(0.5)
            data = self.nrfjprog.rtt_read(0, 200)
            (_, _, res_low, res_mid, res_hi,
             board_id, _, _, _, vdd, _, _) = self._get_metadata(data)

            self.calibration_data = [res_low, res_mid, res_hi]
            self.board_id = board_id
            self.m_vdd = int(vdd)
            self.connected = True
        except Exception as ex:
            raise PPKError("Could not read Board ID, %s" % str(ex))
        return self.calibration_data

    def rtt_stop(self):
        self.nrfjprog.rtt_stop()

    def reset_connection(self):
        self.nrfjprog.rtt_stop()
        self.nrfjprog.sys_reset()
        self.nrfjprog.go()
        self.nrfjprog.rtt_start()
        while not self.nrfjprog.rtt_is_control_block_found:
            continue
        self.write_stuffed([RTTCommand.RUN])
        self.write_stuffed([RTTCommand.AVG_NUM_SET, 0x00, 1])
        return True

    def dut_power_on(self):
        self.log("DUT power on")
        self.write_stuffed([RTTCommand.DUT, 1])

    def dut_power_off(self):
        self.log("DUT power off")
        self.write_stuffed([RTTCommand.DUT, 0])

    def trigger_acquisition_time_set(self, acqtime):
        """ Set the acquisition window in ms
            This is the dataset transferred upon every trigger
            from the PPK.
        """
        self.trigger_buffer_size = int(acqtime/SAMPLE_INTERVAL_MS + 1)
        buffer_size_high = (self.trigger_buffer_size >> 8) & 0xFF
        buffer_size_low = self.trigger_buffer_size & 0xFF
        self.write_stuffed([RTTCommand.TRIG_WINDOW_SET,
                            buffer_size_high, buffer_size_low])
        self.log("Set acqusition time %d (buffer size:%d)" %
                 (acqtime, self.trigger_buffer_size))

    def trigger_value_set(self, trigger):
        """ Set the trigger value in uA
            The trigger will send data if this value is reached
        """
        high = (trigger >> 16) & 0xFF
        mid = (trigger >> 8) & 0xFF
        low = trigger & 0xFF
        self.write_stuffed([RTTCommand.TRIGGER_SET, high, mid, low])
        self.log("Trigger set to %d" % trigger)

    def trigger_stop(self):
        self.write_stuffed([RTTCommand.TRIG_STOP])

    def average_acquisition_time_set(self, milliseconds=0):
        """ Set the aquisition time in milliseconds
            The time an average consumption should use before finishing

            param @milliseconds Timeout value in ms
                                0 = infinite
        """
        self.avg_timeout = milliseconds

    def average_measurement_start(self):
        self.log("Starting average measurement...")
        self.write_stuffed([RTTCommand.RUN])

    def average_measurement_stop(self):
        self.log("Stopping average measurement")
        self.write_stuffed([RTTCommand.STOP])
        self.clear_measurement_data(self.DATA_TYPE_AVERAGE)

    def vdd_set(self, vdd):
        self.log("Setting VDD to %d" %vdd)

        # Setting voltages above or below these values can cause the emu connection to stall.
        if (VDD_SET_MIN > vdd) or (VDD_SET_MAX < vdd):
            raise PPKError("Invalid vdd given to vdd_set (%d)." % vdd)

        target_vdd = vdd
        while True:
            if target_vdd > self.m_vdd:
                new = self.m_vdd + 100 if abs(target_vdd - self.m_vdd) > 100 else target_vdd
            else:
                new = self.m_vdd - 100 if abs(target_vdd - self.m_vdd) > 100 else target_vdd
            vdd_high_byte = new >> 8
            vdd_low_byte = new & 0xFF
            self.write_stuffed([RTTCommand.SETVDD, vdd_high_byte, vdd_low_byte])
            self.m_vdd = new
            self.log("VDD set")

            # A short delay between calls to write_stuffed improves stability.
            if self.m_vdd == target_vdd:
                break
            else:
                time.sleep(0.25)

    def clear_user_resistors(self):
        self.write_stuffed([RTTCommand.CLEAR_RES_USER])

    def clear_cal_resistors(self):
        print("Clearing calibration resistors")
        self.write_stuffed([RTTCommand.CLEAR_RES_CAL])

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
        #     self.log("Changing resistor value")
        #     resistors[1] = 32.4

        print("writing %.3f, %.3f, %.3f" %(resistors[0],
                                           resistors[1],
                                           resistors[2]))
        # Pack the floats
        bufr1 = struct.pack('f', resistors[0])
        bufr2 = struct.pack('f', resistors[1])
        bufr3 = struct.pack('f', resistors[2])
        # PPK receives byte packages, put them in a list
        for b in bufr1:
            r1_list.append(b)
        for b in bufr2:
            r2_list.append(b)
        for b in bufr3:
            r3_list.append(b)

        # Write the floats to PPK
        self.write_stuffed([RTTCommand.SET_RES,
                            (r1_list[0]), (r1_list[1]), (r1_list[2]), (r1_list[3]),
                            (r2_list[0]), (r2_list[1]), (r2_list[2]), (r2_list[3]),
                            (r3_list[0]), (r3_list[1]), (r3_list[2]), (r3_list[3])
                           ])

    def measurement_readout_start(self):
        try:
            self.timeout = threading.Thread(target=self.t_read)
            self.timeout.setDaemon(True)
            self.timeout.start()
            while not self.nrfjprog.rtt_is_control_block_found():
                continue
            self.log("Starting readout")
            self.alive = True
            self.read_thread_start()
        except:
            raise

    def measurement_readout_stop(self):
        self.log("Stopping readout")
        self.alive = False
        self.nrfjprog.rtt_stop()

    def _stream_handler(self, data):
        """ Function called when data ready on RTT."""
        # adc_val = 0
        if len(data) == 4:
            # print('average received')
            f = struct.unpack('f', bytearray(data))[0]
            if math.isnan(f):
                self.log("Test failed, no data received from board")
                raise PPKError("Got invalid data from average set")
            self.average_data_handler(f)

        # elif (len(data) == 5):
        #     print("timestamp received: " + data)

        # elif (len(data) == 6):
        #     print("timestamp received: " + data)

        # else:
        #     print("===================trigger======================")
        #     for i in range(0, len(data), 2):
        #         if (i + 1) < len(data):
        #             tmp = np.uint16((data[i + 1] << 8) + data[i])
        #             self.current_meas_range = (tmp & MEAS_RANGE_MSK) >> MEAS_RANGE_POS
        #             adc_val = (tmp & MEAS_ADC_MSK) >> MEAS_ADC_POS
        #             self.trigger_data_handler(adc_val, self.current_meas_range)

    def average_data_handler(self, data):
        """ Gets called when average data is available

            @int data   Average sample in microamps
        """
        self.measurement_data_callback(data, self.DATA_TYPE_AVERAGE)

    def trigger_data_handler(self):
        adc_val = 0

        if self.trigger_data_captured >= self.trigger_buffer_size:
            self.log("Trigger buffer full")
            self.trigger_stop()
            self.alive = False
            # TODO: What is this?
            # self.data_callback(self.trigger_buffer, self.DATA_TYPE_TRIGGER)
            self.trigger_data_captured = 0
            self.trigger_buffer = []
        else:
            self.trigger_data_captured += 1
            if self.current_meas_range == MEAS_RANGE_LO:
                sample_ua = adc_val * (ADC_REF / (ADC_GAIN * ADC_MAX * MEAS_RES_LO))
            elif self.current_meas_range == MEAS_RANGE_MID:
                sample_ua = adc_val * (ADC_REF / (ADC_GAIN*ADC_MAX*MEAS_RES_MID))
            elif self.current_meas_range == MEAS_RANGE_HI:
                sample_ua = adc_val * (ADC_REF / (ADC_GAIN*ADC_MAX*MEAS_RES_HI))
            elif self.current_meas_range == MEAS_RANGE_INVALID:
                self.log("Range INVALID")
            elif self.current_meas_range == MEAS_RANGE_NONE:
                self.log("Range not detected")
            self.trigger_buffer.append(sample_ua)

    def write_stuffed(self, cmd):
        """Addes escape characters to cmd and then writes it to RTT."""
        try:
            buf = []
            buf.append(STX)
            for byte in cmd:
                if byte in (STX, ETX, ESC):
                    buf.append(ESC)
                    buf.append(byte ^ 0x20)
                else:
                    buf.append(byte)
            buf.append(ETX)
            try:
                print("rtt write initiated")
                self.nrfjprog.rtt_write(0, buf, encoding=None)
                print("rtt write finished")
            except Exception as ex:
                print("Failed write")
                raise ex
        except Exception as ex:
            print("Failed write stuffed")
            raise ex

    def read_thread_start(self):
        # Start thread for reading rtt.
        self.read_thread = threading.Thread(target=self.t_read)
        self.read_thread.setDaemon(True)
        self.read_thread.start()

    def handle_bytes(self, byte):
        """Deals with escape characters and adds to data_buffer."""
        # print('Handle byte: ' + str(byte))
        if self.read_mode == MODE_RECV:
            # Mode Receiving - Receiving data
            if byte == ESC:
                # print('escape received')
                self.read_mode = MODE_ESC_RECV
            elif byte == ETX:
                # print('etx received')
                self._stream_handler(self.data_buffer)
                self.data_buffer[:] = []
                self.read_mode = MODE_RECV
            else:
                # print('append byte')
                self.data_buffer.append(byte)

        elif self.read_mode == MODE_ESC_RECV:
            # Mode Escape Received - Convert next byte
            # print('escing byte')
            self.data_buffer.append(byte ^ 0x20)
            self.read_mode = MODE_RECV

    def t_read(self):
        try:
            while True:
                data = self.nrfjprog.rtt_read(0, 100, encoding=None)
                if data != '':
                    for byte in data:
                        # print('RAW: ' + str(byte))
                        self.handle_bytes(byte)
        except Exception as ex:
            # print("RTT read failed: %s" % str(ex))
            self.alive = False
            raise ex

    def measurement_data_callback(self, data, dtype):
        if dtype == self.DATA_TYPE_AVERAGE:
            self.avg_buffer.append(data)

    def get_measurement_data(self, dtype):
        if dtype == self.DATA_TYPE_AVERAGE:
            return self.avg_buffer
        if dtype == self.DATA_TYPE_TRIGGER:
            return self.trigger_buffer

    def clear_measurement_data(self, dtype):
        if dtype == self.DATA_TYPE_AVERAGE:
            self.avg_buffer = []
        if dtype == self.DATA_TYPE_TRIGGER:
            self.trigger_buffer = []
