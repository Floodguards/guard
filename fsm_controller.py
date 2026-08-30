# ============================================================
# fsm_controller.py
#
# 2026-08-25 갱신: 정연(B)이 VS Code 디버거 화면을 스크린샷으로
# 보내줘서 실제 fsm_controller.py 코드를 확인했다. 아래는 그
# 실제 코드(FloodguardFsm 클래스, _is_sensor_valid/_decide_state/
# _rank/update)를 최대한 그대로 옮긴 것이다. main.py와의 호환을
# 위해 맨 아래에 예전과 같은 시그니처의 update() 함수로 감싸뒀다.
#
# !! 실제 코드를 보고 나서 바로잡은 부분 (이전 재구성과 달랐던 점) !!
# 1) 히스테리시스 방식이 달랐다. 예전 재구성은 "MID/HIGH에서 조건에
#    따라 MID로만 내려간다"는 복잡한 규칙이었는데, 실제 코드는
#    _rank()로 위험도 순위를 매겨서 "새로 계산된 상태의 순위가 현재
#    상태보다 낮으면 그냥 유지"하는 단순한 방식이다 (한 번 HIGH가
#    되면 h_out이 낮아져도 계속 HIGH로 유지됨 - 디버거에서
#    h_out=7.0cm으로 낮아졌는데도 state=HIGH로 유지되는 게 확인됨).
# 2) _is_sensor_valid는 롤/피치 기울기만 확인한다 (max(|roll|,|pitch|)
#    <= invalid_tilt_deg). h_out 음수 체크나 imu_valid 플래그 체크는
#    실제 코드에 없었다 - 예전 재구성에서 내가 임의로 추가한 것.
# 3) h_out_cm이 None인 경우에 대한 방어 코드가 없다 - 실제 코드는
#    h_out_cm이 항상 유효한 숫자라고 가정한다. h_in_cm은 애초에
#    _decide_state에서 아예 쓰지 않는다 (F_net 계산은 pressure_balance.py의
#    몫이고, FSM 단계 판단 자체는 h_out_cm/rise_rate_cm_s/기울기만 봄).
#
# 2026-08-25 추가: ESCAPE를 FloodState의 다섯 번째 상태로 승격함
# (서연 결정 - main.py에서 output_state로 따로 관리하지 않고, FSM
# 자체가 관리하는 정식 상태로 두는 게 보기 편하다고 판단). ESCAPE는
# 센서값으로 직접 도달하는 상태가 아니라 - _decide_state()는 여전히
# IDLE/LOW/MID/HIGH만 계산한다 - "창문이 실제로 열렸다"는 외부
# 이벤트(pressure_balance의 개방 판정)로만 도달한다. 랭크를 가장
# 높게(4) 둬서, 한 번 ESCAPE가 되면 기존 히스테리시스 규칙
# (_rank(new_state) >= _rank(self.state)만 갱신) 그대로 다시 내려가지
# 않는다. main.py는 개방 판정이 나는 순간 escalate_to_escape()를
# 호출해서 이 상태로 승격시킨다.
#
# 2026-08-25 추가 두 번째: _decide_state()를 유나(A)의 "8.25" 작업일지
# 페이지(FLOODGUARD 센서 입력 및 데이터 연동 설계) 12절 decide_state()
# 로직으로 교체함 (서연 결정 - "유나가 롤/피치를 분리해서 계산했으니
# 이걸로 따라간다"). 이전 버전(정연 실제 코드, 디버거로 검증)과 다른 점:
#   1) h_in_cm을 이제 MID/HIGH 판단에 직접 사용한다 (이전엔 h_in_cm을
#      _decide_state에서 아예 안 썼다 - h_in이 미구현이라 항상 None/0
#      취급이었기 때문). h_in_cm이 이제 실제로 측정되면서(HC-SR04P를
#      내부용으로 새로 추가) 의미 있는 입력이 됨.
#   2) roll/pitch를 각각 따로 확인한다 (이전엔 tilt_deg 하나만 계산해서
#      두 값에 똑같이 넣는 임시방편이었다).
#   3) "센서 무효" 처리가 두 단계로 나뉜다: imu_valid=False면 이전
#      상태 유지, sonar_valid=False(기울기 20도 초과)면 이전 상태
#      유지 - 이전 버전은 한 가지 기준(8도)으로만 판단했다.
#   4) severe_tilt(60도 이상) + h_out>=1cm이면 다른 조건보다 먼저
#      HIGH로 즉시 승격한다 (전복 위험 우선 처리).
# !! 주의 !! 이 로직은 정연(B)의 실제 코드가 아니라 유나(A)의 설계
# 문서에 있는 코드다 - 디버거로 검증된 적은 없다. 정연이 자기
# fsm_controller.py에도 이 로직을 반영했는지 확인 필요.
#
# 아래 임계값들은 전부 잠정치다. 유나 8.25 문서 원문: "1cm, 6cm, 14cm,
# 0.3cm/s, 20도, 60도는 검증된 실제 차량 안전기준이 아니라 30cm 높이
# 모형의 초기 실험값이다. 수조 실험에서 측정된 CSV를 기반으로 수정한다."
# window_width_m/pressure_threshold_n(pressure_balance.py)과 같은
# 성격의 TODO라서 아래 표로 한 번에 정리해뒀다 (README 10번도 참고):
#
# !! TODO(잠정치 - 수조 실험 CSV로 보정 예정) !!
# ┌─────────────────────────┬────────┬──────────────────────────┐
# │ 값                       │ 임시값 │ 의미                      │
# ├─────────────────────────┼────────┼──────────────────────────┤
# │ low_level_cm             │ 1cm    │ IDLE→LOW 전환 h_out 기준   │
# │ mid_level_out_cm         │ 6cm    │ MID 전환 h_out 기준        │
# │ mid_level_in_cm          │ 1cm    │ MID 전환 h_in 기준         │
# │ mid_rise_in_cm_s         │ 0.3cm/s│ MID 전환 h_in 상승속도 기준│
# │ high_level_cm            │ 14cm   │ HIGH 전환 h_out/h_in 기준  │
# │ sonar_valid_tilt_deg     │ 20도   │ 초음파 신뢰 가능 기울기 한계│
# │ severe_tilt_deg          │ 60도   │ 전복 위험 판정 기울기       │
# └─────────────────────────┴────────┴──────────────────────────┘
# ============================================================

from dataclasses import dataclass, field
from enum import Enum


class FloodState(Enum):
    IDLE = "IDLE"
    LOW = "LOW"
    MID = "MID"
    HIGH = "HIGH"
    ESCAPE = "ESCAPE"


@dataclass
class SensorData:
    h_out_cm: float
    h_in_cm: float = 0.0
    rise_rate_out_cm_s: float = 0.0
    rise_rate_in_cm_s: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    sonar_valid: bool = True
    imu_valid: bool = True
    severe_tilt: bool = False


@dataclass
class FsmThresholds:
    # !! TODO(잠정치 - 수조 실험 CSV로 보정 예정, 위 표 참고) !!
    low_level_cm: float = 1.0
    mid_level_out_cm: float = 6.0
    mid_level_in_cm: float = 1.0
    mid_rise_in_cm_s: float = 0.3
    high_level_cm: float = 14.0
    sonar_valid_tilt_deg: float = 20.0
    severe_tilt_deg: float = 60.0


@dataclass
class FsmDecision:
    state: FloodState
    reason: str


_STATE_RANK = {
    FloodState.IDLE: 0,
    FloodState.LOW: 1,
    FloodState.MID: 2,
    FloodState.HIGH: 3,
    FloodState.ESCAPE: 4,
}


class FloodguardFsm:
    def __init__(self, thresholds: FsmThresholds = None):
        self.thresholds = thresholds or FsmThresholds()
        self.state = FloodState.IDLE

    def _decide_state(self, data: SensorData) -> tuple:
        """2026-08-25 교체: 유나(A) "8.25" 문서 12절 decide_state() 로직을
        옮긴 것. 조건 자체(임계값)는 문서 pseudocode 그대로다.

        2026-08-25 다섯 번째 갱신: 검사 순서는 실제 code_A.zip의
        fsm.py 순서로 맞췄다 (severe_tilt 먼저, 그다음 imu_valid,
        그다음 sonar_valid) - 원래는 문서 pseudocode 순서(imu_valid를
        가장 먼저 봄)를 따랐는데, sensor_input.py에서 severe_tilt를
        `imu_valid and (...)`로 계산하기 때문에(즉 imu_valid=False면
        severe_tilt는 항상 False) 어느 순서로 봐도 최종 판정 결과는
        동일하다. 결과가 같으므로 실제 코드 순서를 따르기로 함."""
        t = self.thresholds

        # 1) 전복 위험(severe_tilt)이면 다른 조건보다 먼저 HIGH로 즉시 승격
        #    (severe_tilt는 imu_valid=True일 때만 True가 될 수 있으므로
        #    이 체크가 imu_valid 체크보다 앞에 와도 결과는 같다)
        if data.severe_tilt and data.h_out_cm >= t.low_level_cm:
            return FloodState.HIGH, "severe_tilt_emergency_high"

        # 2) IMU 자체가 무효면 이전 상태 유지
        if not data.imu_valid:
            return self.state, "imu_invalid_hold_previous_state"

        # 3) 초음파 신뢰 불가(기울기 20도 초과)면 이전 상태 유지
        if not data.sonar_valid:
            return self.state, "sonar_invalid_hold_previous_state"

        # 4) 바깥/안쪽 수위 중 하나라도 HIGH 기준 이상이면 HIGH
        if data.h_out_cm >= t.high_level_cm or data.h_in_cm >= t.high_level_cm:
            return FloodState.HIGH, "h_out_or_h_in_reached_high_level"

        # 5) 바깥 수위, 안쪽 수위, 안쪽 상승속도 중 하나라도 MID 기준이면 MID
        if (
            data.h_out_cm >= t.mid_level_out_cm
            or data.h_in_cm >= t.mid_level_in_cm
            or data.rise_rate_in_cm_s >= t.mid_rise_in_cm_s
        ):
            return FloodState.MID, "mid_condition_met"

        if data.h_out_cm >= t.low_level_cm:
            return FloodState.LOW, "h_out_reached_low_level"

        # 아무것도 해당 안되면 idle
        return FloodState.IDLE, "no_water_detected"

    @staticmethod
    def _rank(state: FloodState) -> int:
        return _STATE_RANK[state]

    def update(self, data: SensorData) -> FsmDecision:
        new_state, reason = self._decide_state(data)

        if self._rank(new_state) >= self._rank(self.state):
            self.state = new_state

        return FsmDecision(state=self.state, reason=reason)

    def escalate_to_escape(self) -> FsmDecision:
        """2026-08-25 추가: 창문이 실제로 열린 순간 main.py가 호출한다.
        _decide_state()는 센서값으로 ESCAPE를 계산하지 않으므로, 이
        메서드가 ESCAPE로 가는 유일한 경로다. ESCAPE가 랭크 4로 가장
        높아서, 같은 hold-max 규칙으로 한 번 승격되면 다시 내려가지
        않는다 (이미 ESCAPE 상태라면 그대로 유지, 아무 부작용 없음)."""
        if self._rank(FloodState.ESCAPE) >= self._rank(self.state):
            self.state = FloodState.ESCAPE
        return FsmDecision(state=self.state, reason="window_opened_escalate_to_escape")


# ============================================================
# main.py 호환용 래퍼 (예전 함수형 시그니처 유지)
# ============================================================

_fsm = FloodguardFsm()


def update(
    h_out_cm, h_in_cm, rise_rate_cm_s=0.0, roll_deg=0.0, pitch_deg=0.0,
    imu_valid=True, rise_rate_in_cm_s=0.0, sonar_valid=True, severe_tilt=False,
):
    """센서값으로 위험 단계를 판단한다. (state, reason, sensor_valid) 반환.

    2026-08-25 갱신: 유나의 새 sensor_input.py가 rise_rate_in_cm_s/
    sonar_valid/severe_tilt를 함께 넘겨주므로 파라미터를 추가했다.
    rise_rate_cm_s(외부 상승속도)는 이제 _decide_state에서 직접 쓰이지
    않지만 main.py/logger.py 호환을 위해 인자는 남겨뒀다.

    실제 FloodguardFsm은 h_out_cm이 항상 유효한 숫자라고 가정한다
    (None 방어 코드 없음). h_out_cm이 None이면 여기서 걸러서 이전
    상태를 유지한다 - 이건 병합 시 추가한 안전장치다."""
    if h_out_cm is None:
        return _fsm.state.value, "h_out_unavailable", False

    data = SensorData(
        h_out_cm=h_out_cm,
        h_in_cm=h_in_cm if h_in_cm is not None else 0.0,
        rise_rate_out_cm_s=rise_rate_cm_s or 0.0,
        rise_rate_in_cm_s=rise_rate_in_cm_s or 0.0,
        roll_deg=roll_deg or 0.0,
        pitch_deg=pitch_deg or 0.0,
        sonar_valid=sonar_valid,
        imu_valid=imu_valid,
        severe_tilt=severe_tilt,
    )
    decision = _fsm.update(data)
    sensor_valid = decision.reason not in (
        "imu_invalid_hold_previous_state", "sonar_invalid_hold_previous_state",
    )
    return decision.state.value, decision.reason, sensor_valid


def escalate_to_escape():
    """main.py 호환용 함수형 래퍼. 창문이 열린 순간 호출하면 FSM을
    ESCAPE로 승격시키고 (state, reason)을 반환한다."""
    decision = _fsm.escalate_to_escape()
    return decision.state.value, decision.reason
