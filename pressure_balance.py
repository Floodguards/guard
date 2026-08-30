from dataclasses import dataclass

@dataclass(frozen=True)
#수압 계산에 필요한 값들
class PressureConfig:
    water_den_kg_m3=1000.0 #물 밀도
    gravity_m_s2=9.81 #중력가속도
    window_width_m=0.4 #창문/패널 폭(수조 실험 후 보정 필요)
    pressure_threshold_n=57.7 #개방 가능하다고 판단하는 수압차의 기준

@dataclass(frozen=True)
#판단 결과
class PressureDecision:
    can_open:bool
    f_net_n:float #계산된 수압차(단위 N)
    reason:str #판단 이유

#cm->m
def cm_to_m(value_cm: float)->float:
    return value_cm/100.0

#calculate fx
def calc_f_net(
        h_out_cm: float,
        h_in_cm: float,
        config: PressureConfig | None=None,
) -> float:
    config=config or PressureConfig()

    h_out_m=cm_to_m(h_out_cm)
    h_in_m=cm_to_m(h_in_cm)

    level_term=h_out_m**2 - h_in_m**2
    level_term=max(0.0, level_term) #음수 힘 방지

    f_net=(
        0.5
        * config.water_den_kg_m3
        * config.gravity_m_s2
        * config.window_width_m
        * level_term
    )

    return f_net

#f_net이 기준값 이하인지 확인
def is_pressure_balanced(
        f_net_n: float,
        config: PressureConfig | None=None
)-> bool:
    config = config or PressureConfig()
    return f_net_n <= config.pressure_threshold_n

#최종 판단
def should_open_window(
        state: str,
        h_out_cm: float,
        h_in_cm: float,
        config: PressureConfig | None=None,
) -> PressureDecision:
    config=config or PressureConfig()
    f_net_n=calc_f_net(h_out_cm, h_in_cm, config)

    if state == "IDLE":
        return PressureDecision(
            can_open=False,
            f_net_n=f_net_n,
            reason="idle_no_open",
        )

    if state == "ESCAPE":
        return PressureDecision(
            can_open=False,
            f_net_n=f_net_n,
            reason="escape_already_open",
        )

    if state == "LOW":
        return PressureDecision(
            can_open=True,
            f_net_n=f_net_n,
            reason="low_immediate_open",
        )
        
    #mid, high에선 수압차가 충분히 작아졌을 때만 열기
    if state in ("MID", "HIGH") :
        if is_pressure_balanced(f_net_n, config):
            return PressureDecision(
                can_open=True,
                f_net_n=f_net_n,
                reason="pressure_balanced_open",
            )
        
        return PressureDecision(
            can_open=False,
            f_net_n=f_net_n,
            reason="wait_pressure_balance",
        )
        
    return PressureDecision(
        can_open=False,
        f_net_n=f_net_n,
        reason="unknown_state_no_open",
    )
    
#테스트용 샘플
def run_demo() -> None:
    samples = [
        ("IDLE", 0.0, 0.0),
        ("LOW", 2.0, 0.0),
        ("MID", 8.0, 1.0),
        ("HIGH", 15.0, 2.0),
        ("HIGH", 15.0, 12.0),
        ("ESCAPE", 15.0, 12.0),
    ]

    for state, h_out_cm, h_in_cm in samples:
        decision = should_open_window(
            state=state,
            h_out_cm=h_out_cm,
            h_in_cm=h_in_cm,
        )

        print(
            f"state={state:<4} "
            f"h_out={h_out_cm:>5.1f}cm "
            f"h_in={h_in_cm:>5.1f}cm "
            f"f_net={decision.f_net_n:>6.2f}N "
            f"can_open={decision.can_open} "
            f"reason={decision.reason}"
        )


if __name__ == "__main__":
    run_demo()
        
        

    
