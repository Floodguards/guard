import time

from fsm_controller import FloodguardFsm, SensorData
from logger import CsvLogger, LogRow
from pressure_balance import should_open_window
from relay_controller import RelayConfig, RelayController
from serial_sender import SerialConfig, SerialStateSender


# 실험 전 A 파트 센서 코드 대신 사용하는 샘플값
def get_sample_sensor_data() -> list[SensorData]:
    return [
        SensorData(h_out_cm=0.0),
        SensorData(h_out_cm=1.5, rise_rate_cm_s=0.1),
        SensorData(h_out_cm=4.0, rise_rate_cm_s=0.7),
        SensorData(h_out_cm=8.0, h_in_cm=1.0, rise_rate_cm_s=0.3),
        SensorData(h_out_cm=15.0, h_in_cm=2.0, rise_rate_cm_s=0.4),
        SensorData(h_out_cm=7.0, h_in_cm=5.0, rise_rate_cm_s=0.1),
    ]


# 전체 B 파트 통합 테스트
def run_demo() -> None:
    fsm = FloodguardFsm()
    sender = SerialStateSender(
        SerialConfig(
            dry_run=True,
        )
    )
    relay = RelayController(
        RelayConfig(
            max_run_s=0.5,
            dry_run=True,
        )
    )
    logger = CsvLogger("floodguard_integration_log.csv")

    sender.connect()
    relay.setup()

    for data in get_sample_sensor_data():
        # 1. 센서값으로 위험 단계 판단
        fsm_decision = fsm.update(data)
        state = fsm_decision.state.value

        # 2. 수압차 기준으로 개방 가능 여부 판단
        pressure_decision = should_open_window(
            state=state,
            h_out_cm=data.h_out_cm,
            h_in_cm=data.h_in_cm,
        )

        # 3. 창문이 열리는 순간 FSM 상태 자체를 ESCAPE로 승격
        if pressure_decision.can_open and state != "ESCAPE":
            fsm_decision = fsm.escalate_to_escape()
            state = fsm_decision.state.value

        # 4. Arduino로 상태 전송
        sender.send_state(state)

        # 5. 개방 가능하면 릴레이 1회 동작
        relay_decision = relay.open_once(pressure_decision.can_open)

        # 6. B 파트 판단/제어 결과와 핵심 센서값 로그 저장
        logger.write_row(
            LogRow(
                timestamp_s=time.time(),
                state=state,
                fsm_reason=fsm_decision.reason,
                sensor_valid=fsm_decision.sensor_valid,
                f_net_n=pressure_decision.f_net_n,
                can_open=pressure_decision.can_open,
                pressure_reason=pressure_decision.reason,
                relay_on=relay_decision.relay_on,
                relay_reason=relay_decision.reason,
                h_out_cm=data.h_out_cm,
                h_in_cm=data.h_in_cm,
                rise_rate_out_cm_s=data.rise_rate_cm_s,
                rise_rate_in_cm_s=data.rise_rate_in_cm_s,
                roll_deg=data.roll_deg,
                pitch_deg=data.pitch_deg,
                sonar_valid=data.sonar_valid,
                severe_tilt=data.severe_tilt,
            )
        )

        print(
            f"state={state:<4} "
            f"h_out={data.h_out_cm:>5.1f}cm "
            f"h_in={data.h_in_cm:>5.1f}cm "
            f"f_net={pressure_decision.f_net_n:>6.2f}N "
            f"can_open={pressure_decision.can_open} "
            f"relay_on={relay_decision.relay_on}"
        )

    sender.close()
    relay.close()


if __name__ == "__main__":
    run_demo()
