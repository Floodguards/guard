# ============================================================
# serial_sender.py
# 2026-08-25: 정연(B)의 실제 코드/실행 로그로 확인.
# 메시지 형식, ascii 인코딩, dedup 로직 모두 일치. port만
# 공식 프로토콜 문서(ttyUSB0) 기준으로 수정함 (정연 코드는 ttyACM0).
# ESCAPE 추가 (2026-08-25) - 없으면 send_state("ESCAPE")가 ValueError.
# ============================================================

import time
from dataclasses import dataclass

try:
    import serial
except ImportError:  # dry-run tests can run without pyserial installed
    serial = None

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
RESET_WAIT_S = 2.0
DRY_RUN = True

VALID_STATES = {"IDLE", "LOW", "MID", "HIGH", "ESCAPE"}
_serial_conn = None
_last_sent_state = None

@dataclass(frozen=True)
class SerialConfig:
    port: str = PORT
    baudrate: int = BAUDRATE
    reset_wait_s: float = RESET_WAIT_S
    dry_run: bool = DRY_RUN


class SerialStateSender:
    """Arduino 상태 전송을 OutputController가 사용할 수 있게 감싼다."""

    def __init__(self, config=None):
        self.config = config or SerialConfig()
        self._serial_conn = None
        self._last_sent_state = None

    def connect(self):
        if self.config.dry_run:
            print("[DRY RUN] Serial connection skipped")
            return
        if serial is None:
            raise RuntimeError("pyserial is required for real serial output")
        self._serial_conn = serial.Serial(
            self.config.port, self.config.baudrate, timeout=1
        )
        time.sleep(self.config.reset_wait_s)

    def send_state(self, state):
        state = state.upper()
        if state not in VALID_STATES:
            raise ValueError(f"Invalid state: {state}")
        if state == self._last_sent_state:
            return False

        message = f"{state}\n"
        if self.config.dry_run:
            print(f"[DRY RUN] send: {message.strip()}")
        else:
            if self._serial_conn is None:
                raise RuntimeError("Serial is not connected. Call connect() first.")
            self._serial_conn.write(message.encode("ascii"))

        self._last_sent_state = state
        return True

    def close(self):
        if self._serial_conn is not None:
            self._serial_conn.close()
            self._serial_conn = None


def init(dry_run=DRY_RUN):
    global _serial_conn
    if dry_run:
        _serial_conn = None
        print("[DRY RUN] Serial connection skipped")
        return
    if serial is None:
        raise RuntimeError("pyserial is required for real serial output")
    _serial_conn = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(RESET_WAIT_S)


def send_state(state, dry_run=DRY_RUN):
    global _last_sent_state

    state = state.upper()

    if state not in VALID_STATES:
        raise ValueError(f"Invalid state: {state}")

    if state == _last_sent_state:
        return False

    message = f"{state}\n"

    if dry_run:
        print(f"[DRY RUN] send: {message.strip()}")
    else:
        if _serial_conn is None:
            raise RuntimeError("Serial is not connected. Call connect() first.")
        _serial_conn.write(message.encode("ascii"))

    _last_sent_state = state
    return True


def close():
    global _serial_conn
    if _serial_conn is not None:
        _serial_conn.close()
        _serial_conn = None
