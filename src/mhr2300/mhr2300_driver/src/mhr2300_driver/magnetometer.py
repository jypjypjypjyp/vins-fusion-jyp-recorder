import binascii
import os
import re
import sys
import time
import numpy
import rospy
import serial

from mhr2300_msgs.msg import Magnetometer3A

# HMR
class HMR2300_API:

    def __init__(self, devid="99"):
        self.__devid = devid

    @property
    def devid(self):
        return self.__devid

    @devid.setter
    def devid(self, devid):
        self.__devid = devid

    @staticmethod
    def devid_cmd():
        return "*99ID\r".encode()

    @property
    def continuous_stream_cmd(self):
        return "*ddC\r".replace("dd", self.__devid).encode()

    @staticmethod
    def esc_cmd():
        return chr(27).encode()

    @property
    def hw_cmd(self):
        return "*ddH\r".replace("dd", self.__devid).encode()

    @property
    def sw_cmd(self):
        return "*ddF\r".replace("dd", self.__devid).encode()

    @property
    def serial_cmd(self):
        return "*dd#\r".replace("dd", self.__devid).encode()

    @property
    def write_enable_cmd(self):
        return "*ddWE\r".replace("dd", self.__devid).encode()

    def baudrate_cmd(self, baudrate):
        """ 19200 correspond to F and 9600 to S """
        if baudrate is 19200:
            baudrate = "F"
        else:
            baudrate = "S"
        return "*99!BR=bd\r".replace("bd", baudrate).encode()

    @property
    def factory_settings_cmd(self):
        return "*ddD\r".replace("dd", self.__devid).encode()

    def sample_rate_cmd(self, rate):
        nnn = [10, 20, 25, 30, 40, 50, 60, 100, 123, 154]
        rate = min(nnn, key=lambda x: abs(x - rate))
        return ("*ddR=" + str(rate) + "\r").replace("dd", self.__devid).encode()

    def format_cmd(self, format):
        if format == 'binary':
            _format = 'B'
        else:
            _format = 'A'
        return "*{0!s}{1!s}\r".format(self.__devid, _format).encode()


class Utils:

    @staticmethod
    def is_int(s):
        is_int = True
        try:
            int(s)
        except (ValueError):
            is_int = False
        return is_int


class Magnetometer:

    def __init__(self, timeout=2, data_format="binary"):
        self.init_sleep = 0.05
        self.__dev_no = rospy.get_param('~dev_no', '/dev/ttyUSB0')
        self.__baudrate = rospy.get_param('~baudrate', 9600)
        self.__timeout = timeout
        self.__format = data_format
        self.__ser = None
        self.__api = HMR2300_API()

    def init_acquisition(self, timed=0):
        try:
            if self.init_com():
                rospy.loginfo("Init com okay !")
                self.publisher = rospy.Publisher("mhr2300_data/magnetometer", Magnetometer3A, queue_size=1)
                if timed == 0:
                    self.infinite_read()
                else:
                    if self.timed_read(timed):
                        self.close_com()
            rospy.loginfo("Erreur de communication")
            exit()
        except KeyboardInterrupt:
            self.close_com()


    def init_com(self):
        """ Send command to get Device ID """
        self.__ser = serial.Serial(
            self.__dev_no, self.__baudrate, timeout=self.__timeout)

        # Stop the Continious Stream, avoid error
        self.__ser.write(self.__api.esc_cmd())
        self.__ser.write(self.__api.devid_cmd())
        tmp = self.__ser.readline().decode()

        # Get Dev ID
        if "ID= " in tmp:
            self.__api.devid = tmp.split("ID= ")[1].replace("\r", "")
            rospy.loginfo(self.__api.devid)

            init_cmds = [self.__api.factory_settings_cmd, self.__api.format_cmd(self.__format),
                         self.__api.sample_rate_cmd(100), self.__api.continuous_stream_cmd]

            for cmd in init_cmds:
                self.__ser.write(self.__api.write_enable_cmd)
                rospy.loginfo(self.__ser.readline().decode())
                time.sleep(self.init_sleep)
                rospy.loginfo(cmd)
                self.__ser.write(cmd)
                if cmd != self.__api.continuous_stream_cmd:
                    rospy.loginfo(self.__ser.readline().decode())
                time.sleep(self.init_sleep)
            return True
        return False

    def infinite_read(self):
        while True:
            self.read_stream()

    def timed_read(self, loop_time=1):
        t_end = time.time() + loop_time

        while time.time() < t_end:
            self.read_stream()

        return True

    def read_stream(self):
        eol = b'\r'
        line = bytearray()
        while True:
            char = self.__ser.read(1)
            if char:
                if char == eol:
                    break
                else:
                    line += char
        tmp = []
        del tmp[:]
        answer = self.parse_xyz(line)
        if answer is not None:
            msg = Magnetometer3A();
            msg.x = answer[0]
            msg.y = answer[1]
            msg.z = answer[2]
            self.publisher.publish(msg)
            

    def parse_xyz(self, answer):
        if answer == b'':
            return None
        if self.__format == "binary":
            answer = binascii.hexlify(answer)
            hexa = [answer[:4], answer[4:8], answer[8:12]]
            try:
                decimal = [int(hexa[0], 16), int(
                    hexa[1], 16), int(hexa[2], 16)]
                for j in range(0, 3):
                    if decimal[j] >= 35536:
                        decimal[j] -= 65536
                    decimal[j] = numpy.around(decimal[j] * 0.006667, 10)
                x, y, z = decimal[0], decimal[1], decimal[2]
            except:
                return None
        else:
            answer = answer.decode("utf-8")
            answer = answer.replace("\r", "")
            x = re.sub("[ ,]", '', answer[:7])
            x = (float(x) / 150)
            y = re.sub("[ ,]", '', answer[9:16])
            y = (float(y) / 150)
            z = re.sub("[ ,]", '', answer[18:25])
            z = (float(z) / 150)

        result = []
        result.append(x)
        result.append(y)
        result.append(z)
        return result

    def close_com(self):
        rospy.loginfo("exit")
        self.__ser.write(self.__api.esc_cmd())
        self.__ser.close()
        exit()
