import time
import socket
import pickle
import numpy as np
from vsp.processor import Processor
from vsp.utils import compose

# Imports required for serial communication with the sensor
from math import *
import serial
import minimalmodbus as mm
import io
import libscrc

class FT_300s:

    def __init__(self, host='****', port=63351):  # Replace sensitive IP with placeholder
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data = []
        self.connect()

    def reset(self):
        self.data = []

    def connect(self):
        try:
            print(f"Connecting FT sensor to {self.host}")
            self.socket.connect((self.host, self.port))
            print("Connection successful")
        except:
            print("Connection failed")

    def read(self, outfile=None):
        ft_data = self.socket.recv(67).decode()
        return [time.time(), [float(x) for x in ft_data[1:-1].split(', ')]]

    def close(self):
        pass


class FTSensorProcessor(Processor):

    def __init__(self, ft_sensor, pipeline=[], view=None, display=None, writer=None):
        self.ft_sensor = ft_sensor
        self.pipeline = pipeline
        self.view = view
        self.display = display
        self.writer = writer

    def process(self, num_frames, outfile=None):
        if len(self.pipeline) > 0:
            pipeline_func = compose(*self.pipeline[::-1])

        if self.display:
            if len(self.pipeline) > 0:
                display_func = compose(self.display.write, self.view.draw)
            else:
                display_func = self.display.write
            self.display.open()

        results = []
        self._cancel = False
        for i in range(num_frames):
            if self._cancel:
                break

            inp = self.ft_sensor.read()

            if len(self.pipeline) > 0:
                out = pipeline_func(inp)
            else:
                out = inp

            if self.display:
                if len(self.pipeline) > 0:
                    display_func(inp, out)
                else:
                    display_func(inp)

            results.append(out)

        if self.display:
            self.display.close()
        if self.writer and outfile:
            with open(outfile, 'wb') as t:
                pickle.dump(results, t)

        return np.array(results)

    def cancel(self):
        self._cancel = True

    def close(self):
        self.ft_sensor.close()


class FTSensorSerial:
    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self.close()

    def close(self):
        pass

    def __init__(self, port="/dev/ttyUSB0", zero_ref=[0, 0, 0, 0, 0, 0]):
        self.PORTNAME = port
        self.BAUDRATE = 19200
        self.BYTESIZE = 8
        self.PARITY = "N"
        self.STOPBITS = 1
        self.TIMEOUT = 1
        self.SLAVEADDRESS = 9
        self.STARTBYTES = bytes([0x20, 0x4e])
        self.zero_ref = zero_ref
        self.__stop_streaming()

    def __forceFromSerialMessage(self, serialMessage):
        forceTorque = [0, 0, 0, 0, 0, 0]
        for i in range(3):
            forceTorque[i] = round(
                int.from_bytes(serialMessage[2 + i*2:4 + i*2], byteorder='little', signed=True) / 100 - self.zero_ref[i], 2
            )
        for i in range(3, 6):
            forceTorque[i] = round(
                int.from_bytes(serialMessage[2 + i*2:4 + i*2], byteorder='little', signed=True) / 1000 - self.zero_ref[i], 2
            )
        return forceTorque

    def __crcCheck(self, serialMessage):
        crc = int.from_bytes(serialMessage[14:16], byteorder='little', signed=False)
        crcCalc = libscrc.modbus(serialMessage[0:14])
        return crc == crcCalc

    def __stop_streaming(self):
        ser = serial.Serial(port=self.PORTNAME, baudrate=self.BAUDRATE, bytesize=self.BYTESIZE,
                            parity=self.PARITY, stopbits=self.STOPBITS, timeout=self.TIMEOUT)
        ser.write(bytearray([0xff] * 50))
        ser.close()

    def set_zero_ref(self):
        mm.BAUDRATE = self.BAUDRATE
        mm.BYTESIZE = self.BYTESIZE
        mm.PARITY = self.PARITY
        mm.STOPBITS = self.STOPBITS
        mm.TIMEOUT = self.TIMEOUT

        ft300 = mm.Instrument(self.PORTNAME, slaveaddress=self.SLAVEADDRESS)
        ft300.close_port_after_each_call = True
        ft300.write_register(410, 0x0200)
        del ft300

        ser = serial.Serial(port=self.PORTNAME, baudrate=self.BAUDRATE, bytesize=self.BYTESIZE,
                            parity=self.PARITY, stopbits=self.STOPBITS, timeout=self.TIMEOUT)

        ser.read_until(self.STARTBYTES)
        data = ser.read_until(self.STARTBYTES)
        data_array = self.STARTBYTES + bytearray(data)[:-2]

        if not self.__crcCheck(data_array):
            raise Exception("CRC ERROR: Serial message and CRC mismatch")

        self.zero_ref = self.__forceFromSerialMessage(data_array)
        self.__stop_streaming()

    def get_ft(self):
        mm.BAUDRATE = self.BAUDRATE
        mm.BYTESIZE = self.BYTESIZE
        mm.PARITY = self.PARITY
        mm.STOPBITS = self.STOPBITS
        mm.TIMEOUT = self.TIMEOUT

        ft300 = mm.Instrument(self.PORTNAME, slaveaddress=self.SLAVEADDRESS)
        ft300.close_port_after_each_call = True
        ft300.write_register(410, 0x0200)
        del ft300

        ser = serial.Serial(port=self.PORTNAME, baudrate=self.BAUDRATE, bytesize=self.BYTESIZE,
                            parity=self.PARITY, stopbits=self.STOPBITS, timeout=self.TIMEOUT)

        ser.read_until(self.STARTBYTES)
        data = ser.read_until(self.STARTBYTES)
        data_array = self.STARTBYTES + bytearray(data)[:-2]

        if not self.__crcCheck(data_array):
            raise Exception("CRC ERROR: Serial message and CRC mismatch")

        self.__stop_streaming()

        return self.__forceFromSerialMessage(data_array)


if __name__ == "__main__":
    ft_sensor = FTSensorSerial()
    ft_sensor.set_zero_ref()

    while True:
        print(ft_sensor.get_ft(), time.time())
        time.sleep(0.1)
