from dataclasses import dataclass
from enum import Enum

#fsm 상태
class FloodState(str, Enum):
    IDLE="IDLE"
    LOW="LOW"
    MID="MID"
    HIGH="HIGH"
    ESCAPE="ESCAPE"

#받을 센서값 형식
@dataclass(frozen=True)
class SensorData:
    #바깥쪽 수위
    h_out_cm: float
    #안쪽 수위, 기본값 0.0
    h_in_cm: float=0.0
    #수위 상승 속도, 1초에 몇 cm씩 올라가는지
    rise_rate_cm_s: float=0.0
    #안쪽 수위 상승 속도
    rise_rate_in_cm_s: float=0.0
    #좌우 기울기
    roll_deg: float=0.0
    #앞뒤 기울기
    pitch_deg: float=0.0
    imu_valid: bool=True
    sonar_valid: bool=True
    severe_tilt: bool=False

#fsm 판단 기준값(임시)
@dataclass(frozen=True)
class FsmThresholds:
    #바깥 수위 1cm 이상이면 low
    low_level_cm: float=1.0
    #바깥 수위 6cm 이상이면 MID
    mid_level_out_cm: float=6.0
    #안쪽 수위 1cm 이상이면 MID
    mid_level_in_cm: float=1.0
    #14cm 이상이면 HIGH
    high_level_cm: float=14.0
    #바깥 수위 상승 속도 기준
    fast_rise_cm_s: float=0.5
    #안쪽 수위 상승 속도 기준
    mid_rise_in_cm_s: float=0.3
    #초음파 신뢰 가능 기울기 한계
    sonar_valid_tilt_deg: float=20.0
    #전복 위험 판정 기울기
    severe_tilt_deg: float=60.0

#fsm 판단 결과
@dataclass(frozen=True)
class FsmDecision:
    #판단된 상태
    state: FloodState
    #왜 그 상태가 되었는지
    reason: str
    #센서값 믿을 수 있?
    sensor_valid: bool

#실제 fsm
class FloodguardFsm:
    def __init__(self, thresholds: FsmThresholds | None=None):
        #사용자가 기준값 넣으면 그거 사용/or 안 넣으면 기본 기준값
        self.thresholds=thresholds or FsmThresholds()
        #처음 상태 IDLE
        self.state=FloodState.IDLE

    #센서값 넣으면 상태 판단
    def update(self, data: SensorData) -> FsmDecision:
        if self.state == FloodState.ESCAPE:
            return FsmDecision(
                state=FloodState.ESCAPE,
                reason="escape_state_latched",
                sensor_valid=True,
            )

        sensor_valid= self._is_sensor_valid(data)
        #센서값 기준으로 다음 상태 판단
        next_state, reason=self._decide_state(data, sensor_valid)
        #상태 갑자기 내려가는 것 방지
        if self._rank(next_state)<self._rank(self.state):
            return FsmDecision(
                state=self.state,
                reason=f"hold_{self.state.value.lower()}_to_avoid_flapping",
                sensor_valid=sensor_valid,
            )

        self.state=next_state
        return FsmDecision(state=next_state, reason=reason, sensor_valid=sensor_valid)

    #fsm 상태 처음으로 되돌리는 함수
    def reset(self)->None:
        self.state=FloodState.IDLE

    #창문이 열리는 순간 ESCAPE 상태로 승격
    def escalate_to_escape(self) -> FsmDecision:
        self.state = FloodState.ESCAPE
        return FsmDecision(
            state=FloodState.ESCAPE,
            reason="window_opened_escape",
            sensor_valid=True,
        )

    #센서값 정상인지 확인
    def _is_sensor_valid(self, data: SensorData)->bool:
        if data.h_out_cm is None or data.h_in_cm is None:
            return False
        if data.h_out_cm<0 or data.h_in_cm<0:
            return False
        if not data.imu_valid:
            return False
        if not data.sonar_valid:
            return False
        #roll, pitch중 더 큰 기울기를 기준으로
        max_tilt=max(abs(data.roll_deg), abs(data.pitch_deg))
        if max_tilt >= self.thresholds.severe_tilt_deg:
            return True
        return max_tilt<=self.thresholds.sonar_valid_tilt_deg

    #상태 판단
    def _decide_state(
            self, data: SensorData, sensor_valid: bool
    )->tuple[FloodState, str]:
        #센서값 비정상이면 이전 상태 유지
        if not sensor_valid:
            return self.state, "invalid_sensor_data_hold_previous_state"
        t=self.thresholds
        max_tilt=max(abs(data.roll_deg), abs(data.pitch_deg))
        severe_tilt = data.severe_tilt or max_tilt >= t.severe_tilt_deg
        if severe_tilt:
            return FloodState.HIGH, "severe_tilt_high_risk"
        #바깥수위 기준치 이상이면 high
        if data.h_out_cm>=t.high_level_cm or data.h_in_cm>=t.high_level_cm:
            return FloodState.HIGH, "water_reached_high_level"
        if data.h_out_cm>=t.mid_level_out_cm:
            return FloodState.MID, "h_out_reached_mid_level"
        if data.h_in_cm>=t.mid_level_in_cm:
            return FloodState.MID, "h_in_reached_mid_level"
        if data.rise_rate_in_cm_s>=t.mid_rise_in_cm_s:
            return FloodState.MID, "h_in_fast_rise_rate"
        #수위가 아직 6cm가 아니더라도 상승속도 빠르면 mid로
        if data.rise_rate_cm_s>=t.fast_rise_cm_s and data.h_out_cm>=t.low_level_cm:
            return FloodState.MID, "fast_rise_rate_after_low_detection"
        if data.h_out_cm >= t.low_level_cm:
            return FloodState.LOW, "h_out_reached_low_level"
        #아무것도 해당x->idle
        return FloodState.IDLE, "no_water_detected"

    @staticmethod
    #상태 위험도 숫자로
    def _rank(state:FloodState)-> int:
        return {
            FloodState.IDLE: 0,
            FloodState.LOW: 1,
            FloodState.MID: 2,
            FloodState.HIGH: 3,
            FloodState.ESCAPE: 4,
        } [state]

    #테스트용 함수
def run_demo()->None:
    fsm=FloodguardFsm()

    samples=[
        SensorData(h_out_cm=0.0),
        SensorData(h_out_cm=1.5, rise_rate_cm_s=0.1),
        SensorData(h_out_cm=4.0, rise_rate_cm_s=0.7),
        SensorData(h_out_cm=8.0, rise_rate_cm_s=0.3),
        SensorData(h_out_cm=15.0, h_in_cm=2.0, rise_rate_cm_s=0.4),
        SensorData(h_out_cm=7.0, h_in_cm=5.0, rise_rate_cm_s=0.1),
    ]

    #샘플 하나씩 넣어서 상태 판단
    for sample in samples:
        decision=fsm.update(sample)
        print(
            f"h_out={sample.h_out_cm:>4.2f}cm "
            f"rate={sample.rise_rate_cm_s:>3.2f}cm/s "
            f"-> {decision.state.value:<4} "
            f"reason={decision.reason}"
        )

if __name__ == "__main__":
    run_demo()
