"""FT300-S serial adapter retaining the legacy frame scaling and Modbus CRC.

Hardware behavior has not been verified. Dependencies are loaded on construction.
Each request restarts streaming, as in the original script; no rate guarantee.
"""


class FTSensorSerial:
    START_BYTES = bytes([0x20, 0x4e])

    def __init__(self, port="/dev/ttyUSB0", zero_ref=None):
        import serial
        import minimalmodbus
        import libscrc
        self._serial = serial
        self._modbus = minimalmodbus
        self._crc = libscrc.modbus
        self.port = port
        self.zero_ref = list(zero_ref) if zero_ref is not None else [0] * 6
        self._connection = None
        self._stop_streaming()

    def _open(self):
        self._connection = self._serial.Serial(
            port=self.port, baudrate=19200, bytesize=8, parity="N", stopbits=1, timeout=1)
        return self._connection

    def close(self):
        if self._connection is not None:
            try:
                self._connection.close()
            finally:
                self._connection = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _stop_streaming(self):
        try:
            self._open().write(bytes([0xff] * 50))
        finally:
            self.close()

    def _frame(self):
        instrument = self._modbus.Instrument(self.port, slaveaddress=9)
        try:
            instrument.serial.baudrate = 19200
            instrument.serial.bytesize = 8
            instrument.serial.parity = "N"
            instrument.serial.stopbits = 1
            instrument.serial.timeout = 1
            instrument.close_port_after_each_call = True
            instrument.write_register(410, 0x0200)
        finally:
            instrument.serial.close()
        try:
            connection = self._open()
            connection.read_until(self.START_BYTES)
            data = connection.read_until(self.START_BYTES)
            frame = self.START_BYTES + data[:-2]
            if len(frame) != 16 or not data.endswith(self.START_BYTES):
                raise RuntimeError("Incomplete FT300-S frame or serial timeout")
            if int.from_bytes(frame[14:16], "little") != self._crc(frame[:14]):
                raise RuntimeError("FT300-S CRC mismatch")
            return frame
        finally:
            self.close()
            self._stop_streaming()

    def _decode(self, frame):
        return [round(int.from_bytes(frame[2 + i * 2:4 + i * 2], "little", signed=True)
                      / (100 if i < 3 else 1000) - self.zero_ref[i], 2) for i in range(6)]

    def set_zero_ref(self):
        self.zero_ref = [0] * 6
        self.zero_ref = self._decode(self._frame())

    def get_ft(self):
        return self._decode(self._frame())
