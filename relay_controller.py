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
MAX_RUN_S = 5.0

_already_opened = False
_relay = None


def init():
    global _relay, _already_opened
    _already_opened = False
    from gpiozero import DigitalOutputDevice
    _relay = DigitalOutputDevice(GPIO_PIN, active_high=ACTIVE_HIGH)
    _relay.off()


def close():
    global _relay
    if _relay is not None:
        _relay.off()
        _relay.close()
        _relay = None


def run(can_open_flag):
    global _already_opened

    if not can_open_flag:
        return {"relay_on": False, "already_opened": _already_opened, "reason": "open_condition_false"}

    if _already_opened:
        return {"relay_on": False, "already_opened": True, "reason": "already_opened_skip"}

    if _relay is None:
        raise RuntimeError("릴레이가 초기화되지 않았습니다. init()을 먼저 호출하세요.")

    _relay.on()
    try:
        time.sleep(MAX_RUN_S)
    finally:
        _relay.off()

    _already_opened = True
    return {"relay_on": True, "already_opened": True, "reason": "relay_open_executed"}


def run_trial_pulse(duration_s):
    """Run one supervised calibration pulse without changing the open latch.

    This deliberately has no relationship to the FSM's automatic-opening
    decision.  There is no position feedback, so the caller must record the
    observed target-position result after the pulse.
    """
    if duration_s <= 0 or duration_s > MAX_RUN_S:
        raise ValueError(f"통전 시간은 0초 초과 {MAX_RUN_S}초 이하여야 합니다.")

    if _relay is None:
        raise RuntimeError("릴레이가 초기화되지 않았습니다. init()을 먼저 호출하세요.")

    _relay.on()
    try:
        time.sleep(duration_s)
    finally:
        _relay.off()
    return {"relay_on": True, "reason": "trial_relay_pulse_complete"}
