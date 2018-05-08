import threading
import time
from pynrfjprog import API

JLINK_PRO_V8 = 4000
JLINK_OBD       = 1000

DEBUG = True
# Always try to have highest speed
JLINK_SPEED_KHZ = JLINK_PRO_V8

STX = 0x02
ETX = 0x03
ESC = 0x1F

MODE_IDLE = 0
MODE_RECV = 1
MODE_ESC_RECV = 2

NRF_EGU0_BASE          = 0x40014000
TASKS_TRIGGER0_OFFSET  = 0
TASKS_TRIGGER1_OFFSET  = 4
TASKS_TRIGGER2_OFFSET  = 8
TASKS_TRIGGER3_OFFSET  = 12
TASKS_TRIGGER4_OFFSET  = 16
TASKS_TRIGGER5_OFFSET  = 20
TASKS_TRIGGER6_OFFSET  = 24
TASKS_TRIGGER7_OFFSET  = 28
TASKS_TRIGGER8_OFFSET  = 32
TASKS_TRIGGER9_OFFSET  = 36
TASKS_TRIGGER10_OFFSET = 40
TASKS_TRIGGER11_OFFSET = 44
TASKS_TRIGGER12_OFFSET = 48
TASKS_TRIGGER13_OFFSET = 52
TASKS_TRIGGER14_OFFSET = 56
TASKS_TRIGGER15_OFFSET = 60

def debug_print(line):
    if(DEBUG):
        print(line)
    else:
        pass


class rtt(object):
    def __init__(self, callback):
        self.alive = True
        # Open connection to debugger and rtt
        self.nrfjprog = API.API('NRF52', log=True)
        self.nrfjprog.open()
        try:
            self.nrfjprog.connect_to_emu_without_snr(jlink_speed_khz=JLINK_SPEED_KHZ)
        except:
            print("\r\nNo emulator connection detected, exiting.")
            exit()
        self.nrfjprog.sys_reset()
        self.nrfjprog.go()
        self.nrfjprog.rtt_start()
        time.sleep(1)

        self.callback = callback

    def start(self):
        # Start thread for reading rtt.
        self.read_thread = threading.Thread(target=self.t_read)
        self.read_thread.setDaemon(True)
        self.read_thread.start()

    def t_read(self):
        print("PPK running")
        try:
            self.read_mode = MODE_IDLE
            data_buffer = []

            while self.alive:
                try:
                    data = self.nrfjprog.rtt_read(0, 10000, encoding=None)

                    if data != '':
                        for byte in data:
                            n = byte
                            if self.read_mode == MODE_IDLE:
                                ''' Mode Idle - Not Receiving '''
                                if n == STX:
                                    self.read_mode = MODE_RECV

                            elif self.read_mode == MODE_RECV:
                                ''' Mode Receiving - Receiving data '''
                                if n == ESC:
                                    self.read_mode = MODE_ESC_RECV
                                elif n == ETX:
                                    self.callback(data_buffer)
                                    data_buffer[:] = []
                                    self.read_mode = MODE_IDLE
                                elif n == STX:
                                    data_buffer[:] = []
                                else:
                                    data_buffer.append(n)

                            elif self.read_mode == MODE_ESC_RECV:
                                ''' Mode Escape Received - Convert next byte '''
                                data_buffer.append(n ^ 0x20)
                                self.read_mode = MODE_RECV
                except Exception as e:
                    print(e)
                    print("Lost connection, retrying for 10 times")
                    print(("Reconnecting..."))
                    connected = False
                    tries = 0
                    while(tries != 10):
                        try:
                            print(tries)
                            time.sleep(0.6)
                            self.nrfjprog.close()
                            self.nrfjprog = API.API('NRF52')
                            self.nrfjprog.open()
                            self.nrfjprog.connect_to_emu_without_snr(jlink_speed_khz=JLINK_SPEED_KHZ)
                            self.nrfjprog.sys_reset()
                            self.nrfjprog.go()
                            self.nrfjprog.rtt_start()
                            time.sleep(1)
                            print("Reconnected, you may start the graphs again.")
                            connected = True
                            break

                        except Exception as e:
                            print(("Reconnecting..."))
                            tries += 1
                            self.alive = connected
                    if (connected):
                        self.alive = True
                    else:
                        raise Exception("Failed to reconnect")

        except Exception as e:
            print(e)
            self.alive = False

    def write_stuffed(self, cmd):
        s = ''
        s = chr(STX)
        for byte in cmd:
            if byte == STX or byte == ETX or byte == ESC:
                s = s + chr(ESC)
                s = s + chr(byte ^ 0x20)
            else:
                s = s + chr(byte)
        s = s + chr(ETX)
        try:
            try:
                debug_print("rtt write initiated")
                self.nrfjprog.rtt_write(0, s, encoding=None)
                debug_print("rtt write finished")
                #time.sleep(1)
            except Exception as e:
                debug_print("write failed, %s") %str(e)

            while(True):
                try:
                    debug_print("write u32 initiated")
                    self.nrfjprog.write_u32(NRF_EGU0_BASE + TASKS_TRIGGER0_OFFSET, 0x00000001, 0)
                    debug_print("write u32 finished")
                    break
                except Exception as e:
                    debug_print("write u32 failed")
                    continue

            while(True):
                try:
                    self.nrfjprog.go()
                    break
                except Exception as e:
                    debug_print("go failed, %s" %str(e))
                    continue
        except:
            pass
