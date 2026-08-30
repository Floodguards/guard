from dataclasses import asdict, dataclass
import time


@dataclass(frozen=True)
class SensorReading:
    outside_raw_distance_cm: float | None
    outside_distance_cm: float | None
    inside_raw_distance_cm: float | None
    inside_distance_cm: float | None
    h_out_cm: float
    h_in_cm: float
    level_difference_cm: float
    rise_rate_cm_s: float
    rise_rate_in_cm_s: float
    roll_deg: float
    pitch_deg: float
    imu_valid: bool
    outside_valid: bool
    inside_valid: bool
    sonar_valid: bool
    severe_tilt: bool


class SensorReader:
    """A파트 실제 센서 연결 전 통합 테스트용 센서 입력 래퍼."""

    def __init__(self):
        self._last_time_s: float | None = None
        self._last_h_out_cm: float | None = None
        self._last_h_in_cm: float | None = None

    def init(self) -> None:
        self._last_time_s = time.monotonic()

    def read_all(self) -> dict:
        now = time.monotonic()
        h_out_cm = 0.0
        h_in_cm = 0.0
        dt = max(now - (self._last_time_s or now), 1e-6)
        rise_rate_cm_s = 0.0 if self._last_h_out_cm is None else (h_out_cm - self._last_h_out_cm) / dt
        rise_rate_in_cm_s = 0.0 if self._last_h_in_cm is None else (h_in_cm - self._last_h_in_cm) / dt

        self._last_time_s = now
        self._last_h_out_cm = h_out_cm
        self._last_h_in_cm = h_in_cm

        reading = SensorReading(
            outside_raw_distance_cm=None,
            outside_distance_cm=None,
            inside_raw_distance_cm=None,
            inside_distance_cm=None,
            h_out_cm=h_out_cm,
            h_in_cm=h_in_cm,
            level_difference_cm=h_out_cm - h_in_cm,
            rise_rate_cm_s=rise_rate_cm_s,
            rise_rate_in_cm_s=rise_rate_in_cm_s,
            roll_deg=0.0,
            pitch_deg=0.0,
            imu_valid=True,
            outside_valid=False,
            inside_valid=False,
            sonar_valid=True,
            severe_tilt=False,
        )
        return asdict(reading)

    def shutdown(self) -> None:
        pass
