from dataclasses import dataclass, asdict
import csv
from pathlib import Path
import time


@dataclass(frozen=True)
class LogRow:
    timestamp_s: float
    state: str
    fsm_reason: str
    sensor_valid: bool
    f_net_n: float
    can_open: bool
    pressure_reason: str
    relay_on: bool
    relay_reason: str
    h_out_cm: float | None = None
    h_in_cm: float | None = None
    rise_rate_out_cm_s: float | None = None
    rise_rate_in_cm_s: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    sonar_valid: bool | None = None
    severe_tilt: bool | None = None


# B 파트 판단/제어 결과만 CSV로 저장하는 클래스
class CsvLogger:
    def __init__(self, path: str = "floodguard_log.csv"):
        self.path = Path(path)
        self.fieldnames = list(LogRow.__dataclass_fields__.keys())
        self._header_written = self.path.exists() and self.path.stat().st_size > 0

    # 처음 저장할 때만 헤더 작성
    def _ensure_header(self) -> None:
        if self._header_written:
            return

        with self.path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writeheader()

        self._header_written = True

    # 로그 한 줄 추가
    def write_row(self, row: LogRow) -> None:
        self._ensure_header()

        with self.path.open("a", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writerow(asdict(row))


# 테스트용 함수
def run_demo() -> None:
    logger = CsvLogger("floodguard_demo_log.csv")

    row = LogRow(
        timestamp_s=time.time(),
        state="MID",
        fsm_reason="h_out_reached_mid_level",
        sensor_valid=True,
        f_net_n=4.67,
        can_open=True,
        pressure_reason="pressure_balanced_open",
        relay_on=True,
        relay_reason="relay_open_executed",
        h_out_cm=8.0,
        h_in_cm=1.0,
        rise_rate_out_cm_s=0.3,
        rise_rate_in_cm_s=0.0,
        roll_deg=0.0,
        pitch_deg=0.0,
        sonar_valid=True,
        severe_tilt=False,
    )

    logger.write_row(row)
    print(f"log_saved={logger.path}")


if __name__ == "__main__":
    run_demo()
