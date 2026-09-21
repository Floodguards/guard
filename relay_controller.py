# ============================================================
# relay_controller.py
# B파트 업로드 relay_controller.py를 기준으로 확인함. 병합용 호출 방식은 별도 조정됨.
# 두 공식 문서 어느 쪽도 릴레이 상세를 규정하지 않아 수정 없음.
# ============================================================

import time

GPIO_PIN = 17
ACTIVE_HIGH = True
# 건식 개방 실측(약 4~5초)을 반영한 임시 통전 상한.
# 종단/위치 피드백이 추가되면 실제 완전 개방 시 즉시 OFF해야 한다.
MAX_RUN_S = 5.5
DRY_RUN = True

_already_opened = False
_relay = None


def init(dry_run=DRY_RUN):
    global _relay, _already_opened
    _already_opened = False
    if dry_run:
        _relay = None
        return
    from gpiozero import DigitalOutputDevice
    _relay = DigitalOutputDevice(GPIO_PIN, active_high=ACTIVE_HIGH)
    _relay.off()


def close():
    global _relay
    if _relay is not None:
        _relay.off()
        _relay.close()
        _relay = None


def run(can_open_flag, dry_run=DRY_RUN):
    global _already_opened

    if not can_open_flag:
        return {"relay_on": False, "already_opened": _already_opened, "reason": "open_condition_false"}

    if _already_opened:
        return {"relay_on": False, "already_opened": True, "reason": "already_opened_skip"}

    if dry_run or _relay is None:
        print(f"[relay_controller][dry_run] 릴레이 ON -> {MAX_RUN_S}s 대기 -> OFF")
    else:
        _relay.on()
        time.sleep(MAX_RUN_S)
        _relay.off()

    _already_opened = True
    return {"relay_on": True, "already_opened": True, "reason": "relay_open_executed"}
