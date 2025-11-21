import serial, threading
from typing import Callable, Optional

class SerialReader:
    def __init__(self, port: str, baud: int, on_line: Callable[[str], None], on_error: Optional[Callable[[Exception], None]]=None):
        self._port = port
        self._baud = baud
        self._on_line = on_line
        self._on_error = on_error
        self._ser = None
        self._stop = threading.Event()
        self._th = None

    def start(self):
        self._ser = serial.Serial(self._port, baudrate=self._baud, timeout=1)
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        try:
            while not self._stop.is_set():
                try:
                    line = self._ser.readline().decode('utf-8', 'ignore')
                    if line:
                        self._on_line(line)
                except Exception as e:
                    if self._on_error:
                        self._on_error(e)
                    break
        finally:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except:  # noqa: E722
                pass

    def write_line(self, s: str):
        if self._ser and self._ser.is_open:
            if not s.endswith('\n'):
                s += '\n'
            self._ser.write(s.encode('utf-8'))

    def stop(self):
        self._stop.set()
        if self._th:
            self._th.join(timeout=1.0)
