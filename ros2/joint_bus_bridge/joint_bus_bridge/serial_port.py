"""Serial port for a USB-RS485 adapter (or the simulator's pseudo-terminal).

Uses pyserial when it is installed (needed for 4 Mbaud on macOS); otherwise plain termios,
which is enough on Linux and for the simulator's pty."""
import os
import termios


class SerialPort:
    def __init__(self, path, baud=4_000_000):
        self._ser = None
        try:
            import serial  # type: ignore
            self._ser = serial.Serial(path, baudrate=baud, timeout=0, write_timeout=0.1)
            return
        except ImportError:
            pass
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0                                   # iflag: raw
        attrs[1] = 0                                   # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                                   # lflag
        speed = getattr(termios, f"B{baud}", None)
        if speed is not None:
            attrs[4] = attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def write(self, data: bytes):
        if self._ser:
            self._ser.write(data)
        else:
            os.write(self.fd, data)

    def read(self) -> bytes:
        if self._ser:
            return self._ser.read(4096)
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""

    def close(self):
        if self._ser:
            self._ser.close()
        else:
            os.close(self.fd)
