"""Pi I2C MPU6050 sampling, independent of the slower water-level loop.

Motion uses abs(ax - baseline_ax) > 0.05 g sustained for at least 10 ms.
Confirmed events retain the FIRST candidate timestamp, excluding confirmation delay.
Timestamps are Pi monotonic times after each read, not hardware interrupt times.
"""

from dataclasses import dataclass
from datetime import datetime
import math
import threading
import time

from sensor_input import _convert_signed_16bit, vector_to_roll_pitch_deg

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus
    except ImportError:
        SMBus = None


@dataclass(frozen=True)
class ImuSample:
    received_monotonic_ns: int
    received_at_iso: str
    ax_g: float
    ay_g: float
    az_g: float
    delta_g: float
    x_delta_g: float = 0.0


@dataclass(frozen=True)
class MotionEvent:
    received_monotonic_ns: int
    received_at_iso: str
    delta_g: float
    x_delta_g: float
    confirmed_monotonic_ns: int
    confirmed_at_iso: str


def add_imu_arguments(parser):
    parser.add_argument('--use-imu', dest='use_imu', action='store_true',
                        default=True, help='Pi I2C IMU 기록 (기본: 활성화)')
    parser.add_argument('--no-imu', dest='use_imu', action='store_false',
                        help='IMU 없이 수위/모터만 실험')


class PiMotionListener:
    INTERVAL_S = 0.002  # Target polling interval; Linux scheduling may delay reads.
    CALIBRATION_INTERVAL_S = 0.005  # Keep startup calibration at about one second.
    CONFIRM_DURATION_NS = 10_000_000
    MOTION_THRESHOLD_G = 0.05
    MAX_SAMPLE_GAP_NS = 6_000_000  # A long unobserved gap cannot confirm persistence.

    def __init__(self):
        self._bus = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._baseline = (0.0, 0.0, 1.0)
        self._latest = None
        self._armed_ns = None
        self._motion = None
        self._count = 0
        self._candidate = None
        self._previous_sample_ns = None
        self._error = ''

    def _read_accel(self):
        data = self._bus.read_i2c_block_data(0x68, 0x3B, 6)
        if len(data) != 6:
            raise OSError('MPU6050 incomplete acceleration sample')
        return tuple(_convert_signed_16bit(data[i], data[i + 1]) / 16384.0
                     for i in (0, 2, 4))

    def start(self):
        if SMBus is None:
            raise RuntimeError('Pi IMU에는 smbus2 또는 smbus가 필요합니다.')
        try:
            self._bus = SMBus(1)
            self._bus.write_byte_data(0x68, 0x6B, 0x00)
            self._bus.write_byte_data(0x68, 0x1C, 0x00)  # +/-2 g
            time.sleep(0.1)
            print('Pi IMU 기준 보정: 약 1초 동안 센서를 움직이지 마세요.')
            samples = []
            for _ in range(200):
                samples.append(self._read_accel())
                time.sleep(self.CALIBRATION_INTERVAL_S)
            self._baseline = tuple(sum(v[i] for v in samples) / len(samples)
                                   for i in range(3))
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    def arm(self, monotonic_ns):
        # Every candidate and confirming sample must be acquired after relay ON.
        with self._lock:
            self._armed_ns = monotonic_ns
            self._motion = None
            self._count = 0
            self._candidate = None
            self._previous_sample_ns = None

    def _record(self, acceleration, read_started_ns, sampled_ns):
        delta = math.sqrt(sum((v - b) ** 2 for v, b in zip(acceleration, self._baseline)))
        x_delta = acceleration[0] - self._baseline[0]
        sample = ImuSample(sampled_ns, datetime.now().isoformat(timespec='milliseconds'),
                           *acceleration, delta, x_delta)
        with self._lock:
            self._latest = sample
            self._error = ''
            if self._armed_ns is not None and read_started_ns >= self._armed_ns:
                if (self._previous_sample_ns is not None
                        and sampled_ns - self._previous_sample_ns > self.MAX_SAMPLE_GAP_NS):
                    self._count = 0
                    self._candidate = None
                self._previous_sample_ns = sampled_ns
                if abs(x_delta) > self.MOTION_THRESHOLD_G:
                    self._count += 1
                    if self._candidate is None:
                        self._candidate = sample
                else:
                    self._count = 0
                    self._candidate = None
                if (self._count >= 3 and self._motion is None
                        and sampled_ns - self._candidate.received_monotonic_ns >= self.CONFIRM_DURATION_NS):
                    candidate = self._candidate
                    self._motion = MotionEvent(
                        candidate.received_monotonic_ns, candidate.received_at_iso,
                        candidate.delta_g, candidate.x_delta_g,
                        sampled_ns, sample.received_at_iso,
                    )

    def _read_loop(self):
        while not self._stop.is_set():
            started_ns = time.monotonic_ns()
            try:
                acceleration = self._read_accel()
                self._record(acceleration, started_ns, time.monotonic_ns())
            except OSError as exc:
                with self._lock:
                    self._latest = None
                    self._count = 0
                    self._candidate = None
                    self._previous_sample_ns = None
                    self._error = str(exc)
            remaining = self.INTERVAL_S - (time.monotonic_ns() - started_ns) / 1e9
            self._stop.wait(max(0, remaining))

    def first_event_after(self, monotonic_ns):
        with self._lock:
            event = self._motion
            return event if event and monotonic_ns is not None and event.received_monotonic_ns >= monotonic_ns else None

    def latest_raw_sample(self):
        with self._lock:
            sample = self._latest
            if sample and time.monotonic_ns() - sample.received_monotonic_ns <= 100_000_000:
                return sample
        return None

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
        if self._bus is not None:
            self._bus.close()
            self._bus = None


def update_imu_data(data, sample):
    """The Pi listener alone owns the IMU bus; water SensorReader has IMU off.

    Angles are absolute accelerometer angles. Movement delta uses startup pose.
    """
    data['imu_valid'] = sample is not None
    data['roll_deg'] = data['pitch_deg'] = None
    if sample is not None:
        data['roll_deg'], data['pitch_deg'] = vector_to_roll_pitch_deg(
            (sample.ax_g, sample.ay_g, sample.az_g))
