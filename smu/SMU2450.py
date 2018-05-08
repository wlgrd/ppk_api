import visa
import time

class API():
    def __init__(self, connect = False):
        # Instrument
        self.smu = None
        # VISA key to the instrument
        self.instrument_identifier = None
        # List of information for the device
        self.info = []
        # Resource manager
        self.rm = visa.ResourceManager()
        if(connect is True):
            self.info = self.discover_and_connect()

    ''' Takes a list of commands and writes them '''
    def _write_list(self, commands):
        for command in commands:
            self.smu.write(command)

    def write(self, cmd):
        if ('?' in cmd):
            try:
                return self.smu.query(cmd)
            except:
                return('No feedback from query')
        self.smu.write(cmd)
        return True

    ''' Prints information about the connected device '''
    def __str__(self):
        if self.smu is not None:
            return(
                'Vendor: {vendor} \r\n'
                'Model: {model} \r\n'
                'Serial: {serial} \r\n'
                'FW Version: {fw_version} \r\n'
                'Instrument identifier: {identifier} \r\n'
                ).format(
                    vendor = self.info[0], model=self.info[1],
                    serial = self.info[2], fw_version = self.info[3],
                    identifier = self.instrument_identifier
                )
        else:
            return('No device connected')

    def _discover(self):
        if self.smu is not None:
            return True
        else:
            devices = self.rm.list_resources()
            for i in devices:
                if '2450' in i:
                    # Get an instance of the instrument resource (GPIBInstrument)
                    self.instrument_identifier = i
                    return True
                else:
                    print(
                        'Available devices: {devices}'.format(devices=devices))
                    return False

    def _connect(self, identifier):
        if self.instrument_identifier is not None:
            self.smu = self.rm.open_resource(identifier)
            # Query does the same as write() with a consecutive read()
            self.info = self.smu.query('*IDN?').split(',')
            return True
        else:
            return False

    def discover_and_connect(self):
        ''' 
            Discovers and connects to Keithley SMU 2450
            If device already connected, returns without reconnecting
            return tuple VENDOR_NAME, MODEL_NAME, SERIAL_NUMBER, FW_VERSION
        '''
        if self._discover() is True:
            if self._connect(self.instrument_identifier) is True:
                return True
            else:
                print("Unable to connect.")
                return False
        else:

            return False

    def is_connected(self):
        return

    def output_enable(self):
        if(self.smu is not None):
            self.smu.write(":OUTP ON")
            return True
        else:
            return False

    def output_disable(self):
        if(self.smu is not None):
            self.smu.write(":OUTP OFF")
            return True
        else:
            return False

    def disable_voltage_source(self):
        ''' Set voltage to 0 '''
        self.write("SOUR:VOLT 0")

    def set_voltage(self, volt):
        ''' Set voltage '''
        print("Voltage set to " + str(volt) + "V")
        self.write("SOUR:VOLT " + str(volt))

    def set_current_range_auto(self):
        self.write("SENS:CURR:RANG:AUTO ON")

    def set_current_limit_uA(self, microamp):
        ''' Set current limit in microamps '''
        if (microamp == 0):
            self.write("SOUR:VOLT:ILIM " + str(10 / 1e9))
            self.set_current_range_uA(0.01)
            self.set_current_limit_uA(0.01)
            return
        # Less than 100nA
        elif (microamp <= 0.1):
            self.set_current_range_uA(0.1)
        # Less than 1uA
        elif (microamp <= 1):
            self.set_current_range_uA(1)
            #self.write("SOUR:VOLT:ILIM " + str(microamp))
        # Less than 10uA
        elif (microamp <= 10):
            self.set_current_range_uA(10)
        # Less than 100uA
        elif (microamp <= 100):
            self.set_current_range_uA(100)
        # Less than 1 mA
        elif (microamp <= 1000):
            self.set_current_range_uA(1000)
        # Less than 10 mA
        elif (microamp <= 10000):
            self.set_current_range_mA(10)
        # Less than 100 mA
        elif (microamp <= 100000):
            self.set_current_range_mA(100)
        # Less than 1 A
        elif (microamp <= 1000000):
            self.set_current_range_mA(1000)

        if (microamp > 6000000):
            raise ValueError("microamp out of range for PPK")
        print("Current limit set to " + str(microamp) + "uA")
        print("Setting " + str(microamp / 1e6))
        self.write("SOUR:VOLT:ILIM " + str(microamp / 1e6))

    def set_current_range_uA(self, microamp):
        valid_values = [0, 0.01, 0.1, 1, 10, 100, 1000, 10000, 100000]
        run_cmd = False
        for i in valid_values:
            if (microamp == i):
                run_cmd = True
                break
        if (run_cmd):
            print("Setting range " + str(microamp / 1e6))
            self.write("SOUR:CURR:RANG " + str(microamp / 1e6))
            return True
        else:
            print("Invalid current range set")
            return False

    def set_current_range_mA(self, milliamp):
        valid_values = [10, 100]
        run_cmd = False
        for i in valid_values:
            if (milliamp == i):
                run_cmd = True
        if (run_cmd):
            self.write("SENS:CURR:RANG " + str(milliamp / 1e3))
            return True
        else:
            print("Invalid current range set")
            return False

    def set_current_limit_nA(self, nanoamp):
        ''' Set current limit in nanoamps '''
        self.write("SOUR:VOLT:ILIM " + str(nanoamp / 1e9))

    def set_current_limit_mA(self, milliamp):
        ''' Set current limit in microamps '''
        # self.write("*RST")
        if (milliamp > 200):
            raise ValueError("milliamp out of range for PPK")
        print("Current limit set to " + str(milliamp) + "mA")
        # self.write("SOUR:VOLT:ILIM " + str(milliamp / 1e3))
        self.set_current_limit_uA(milliamp * 1000)

    def reset_smu(self):
        self.write('*RST')

    def clear_event_registers(self):
        self.write('CLs')

    def save_settings(self):
        self.write('*SAV')
    
    def set_source_current(self):
        self.write(':SOUR:FUNC CURR')

    def set_source_voltage(self):
        self.write(':SOUR:FUNC VOLT')

    def set_current_drain_microamp(self, micro):
        self.write(':SOUR:CURR {amps}'.format(amps = micro/1e6 * -1))

    def read_voltage(self):
        ''' Returns voltage as string with 3 decimals '''
        voltage = (self.write(':READ?'))
        return ('{voltage:.3g}'.format(voltage=float(voltage)))
