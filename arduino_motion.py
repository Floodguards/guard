"""Receive Arduino MPU6050 motion events for the opening-delay tests.

The Arduino continuously emits ``MOTION,<sequence>,<micros>,<delta_g>``.
This module deliberately does not send an arm command: the test accepts only
the first event received after the Pi has commanded the relay ON.
"""

from dataclasses import dataclass
from datetime import datetime
import threading
import time

try:
    import serial
except ImportError:
    serial = None


@dataclass(frozen=True)
class MotionEvent:
    received_monotonic_ns: int
    received_at_iso: str
    sequence: int
    arduino_micros: int
    delta_g: float


class ArduinoMotionListener:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self._connection = None
        self._thread = None
        self._stop_requested = threading.Event()
        self._lock = threading.Lock()
        self._events = []

    def start(self):
        if serial is None:
            raise RuntimeError(
                "Arduino IMU 기록에는 pyserial이 필요합니다. Pi 실행 환경에서 설치하세요."
            )
        self._connection = serial.Serial(self.port, self.baudrate, timeout=0.1)
        # Opening a Uno USB serial port resets it. Do not treat its boot-time
        # serial bytes as measurements.
        time.sleep(2.0)
        self._connection.reset_input_buffer()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop_requested.is_set():
            try:
                line = self._connection.readline().decode("ascii", errors="replace").strip()
            except (serial.SerialException, OSError):
                return
            event = self._parse_motion_line(line)
            if event is not None:
                with self._lock:
                    self._events.append(event)

    @staticmethod
    def _parse_motion_line(line):
        parts = line.split(",")
        if len(parts) != 4 or parts[0] != "MOTION":
            return None
        try:
            sequence = int(parts[1])
            arduino_micros = int(parts[2])
            delta_g = float(parts[3])
        except ValueError:
            return None
        return MotionEvent(
            received_monotonic_ns=time.monotonic_ns(),
            received_at_iso=datetime.now().isoformat(timespec="milliseconds"),
            sequence=sequence,
            arduino_micros=arduino_micros,
            delta_g=delta_g,
        )

    def first_event_after(self, monotonic_ns):
        if monotonic_ns is None:
            return None
        with self._lock:
            for event in self._events:
                if event.received_monotonic_ns >= monotonic_ns:
                    return event
        return None

    def close(self):
        self._stop_requested.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._connection is not None:
            self._connection.close()
            self._connection = None
