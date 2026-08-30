# ============================================================
# serial_sender.py
# 2026-08-25: 정연(B)의 실제 코드/실행 로그로 확인.
# 메시지 형식, ascii 인코딩, dedup 로직 모두 일치. port만
# 공식 프로토콜 문서(ttyUSB0) 기준으로 수정함 (정연 코드는 ttyACM0).
# ESCAPE 추가 (2026-08-25) - 없으면 send_state("ESCAPE")가 ValueError.
# ============================================================

import time

import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
RESET_WAIT_S = 2.0
DRY_RUN = True

VALID_STATES = {"IDLE", "LOW", "MID", "HIGH", "ESCAPE"}

_serial_conn = None
_last_sent_state = None


def init(dry_run=DRY_RUN):
    global _serial_conn
    if dry_run:
        _serial_conn = None
        print("[DRY RUN] Serial connection skipped")
        return
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

    if dry_run or _serial_conn is None:
        print(f"[DRY RUN] send: {message.strip()}")
    else:
        if _serial_conn is None:
            raise RuntimeError("Serial is not connected. Call connect() first.")
        _serial_conn.write(message.encode("ascii"))

    _last_sent_state = state
    return True
