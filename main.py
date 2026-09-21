# ============================================================
# main.py
# A(센서) + B(판단/제어) + C(출력, alarm_output.ino + output_controller.py)
# 를 하나로 연결한 병합 코드. 팀 전체(A/B/C) 코드를 모두 반영한
# 최종 통합 버전이다.
# 2026-08-25: "수압차 로직", "Pi_아두이노_시리얼프로토콜" 공식
# 페이지 기준으로 정리함.
#
# 정연의 원본 main.py는 "A 파트 센서 코드가 아직 연결되지 않아서
# 샘플값으로 테스트한다"고 명시했었다. 이 파일은 그 연결을 실제로
# 해본 버전이다.
#
# 2026-08-25 갱신: 유나가 "8.25" 작업일지 페이지에 새 센서 설계를
# 올리면서, 계속 "가장 급한 문제"였던 h_in_cm 미구현이 해결됐다 -
# 외부(A02YYUW)/내부(HC-SR04P)를 서로 다른 물리 센서로 분리해서 이제
# h_in_cm이 실제로 측정된다 (sensor_input.py 상단 주석 참고). 단
# 이 설계는 아직 정연의 실제 디버거 화면 같은 방식으로 "실행 확인"된
# 적은 없다 - 유나 본인 문서에 있는 코드를 그대로 옮긴 것이라, 실제
# 하드웨어에 연결해서 검증이 필요하다.
#
# 단일 통합 실행 경로: 실제 수위 센서를 읽고 FSM/F_net 판단 후
# Arduino/LCD 출력, 필요 시 릴레이 개방, 통합 CSV 기록을 수행한다.
# 센서만 기록하는 실험은 별도 test2_low_sensor_csv.py를 사용한다.
# FSM 상태 판단 (_decide_state) 로직 자체는 2026-08-25부터 유나의
# 8.25 설계로 교체됐다 (fsm_controller.py 상단 주석 참고).
#
# --- 실행 전 반드시 확인할 미해결 항목 ---
# 1. sensor_input.py의 캘리브레이션(sensor_calibration.json)을 빈
#    수조 상태에서 실제로 실행해서 outside/inside_base_distance_cm,
#    수위 기준값을 채워야 함 (지금은 기본값 30.0)
# 2. fsm_controller.py의 수위/기울기 임계값은 초기 실험값이며 CSV로 보정 필요
# 3. serial_sender.PORT가 /dev/ttyUSB0 맞는지 실제 보드로 확인
# 4. alarm_output.ino를 아두이노에 업로드할 때 실제 배선을
#    R=6/G=9/B=5(PWM), 부저=8, 진동=10으로 맞춰야 함
# 5. output_controller.py는 현재 통합 실행 기준으로 사용 중이다.
#    승현 원본 출력 코드와의 문구·LCD 세부 일치 여부는 별도 확인 필요
#
# --- rollover_detected 로직 삭제 결정 (2026-08-25, 서연) ---
# 이전 "8.16" 버전부터 있던 침수감지 디지털 센서(GPIO27, DO 출력)
# 기반 rollover_detected(전복 판정: 물 감지 + 60도 이상 기울기가
# 1초 유지)는 서연이 원본 문서 페이지에 "이 센서가 필요할까요??"라고
# 남긴 코멘트가 있었고, 실제 code_A.zip의 config.py에도 이 센서 값이
# 없는 걸 확인해서 - 이 센서/로직을 완전히 삭제하기로 함. 전복(기울기)
# 현재 IMU를 사용하지 않으므로 기울기 기반 전복 판정은 비활성화되어 있다.
#
# --- ESCAPE 트리거 결정 (2026-08-25, 서연) ---
# 창문이 실제로 열리는 순간(can_open_flag=True, 즉 LOW 즉시개방이든
# MID/HIGH F_net 조건부 개방이든 상관없이 "지금 열린다"고 판단된 순간,
# 통합 실행 경로에서만 호출한다)
# fsm_controller.escalate_to_escape()를 호출해서 FSM 자체를 ESCAPE로
# 승격시킨다. 전복(기울기, severe_tilt)은 이번 수조 실험(물 유입
# 시나리오)의 핵심이 아니라고 판단해 ESCAPE 트리거에서는 제외했다 -
# 현재 IMU를 사용하지 않으므로 이 경로에서는 severe_tilt가 발생하지 않는다.
#
# 2026-08-25 추가: ESCAPE는 처음엔 main.py가 output_state 변수로만
# 따로 관리했는데("서연의 요구사항 정리" 페이지 참고), 서연이 "그냥
# fsm_controller가 관리하는 같은 상태로 승격하는 게 보기 편하다"고
# 결정해서 fsm_controller.FloodState에 정식 다섯 번째 상태로 추가함.
# 그래서 이제 output_state 변수가 따로 없고, state 하나만 계속
# 갱신해서 쓴다 (lcd_display/logger/print 전부 동일한 state 사용).
#
# 2026-08-25 일곱 번째 갱신: sensor_input이 함수 방식에서
# SensorReader 클래스 방식으로 바뀌어서(sensor_input.py 상단 "변경 5"
# 참고, 서연이 실제 code_A.zip과 구조를 맞추라고 요청함), 여기서도
# sensor_input.init()/read_all() 대신 SensorReader 인스턴스를 만들어
# sensor_reader.init()/sensor_reader.read_all()로 호출하도록 바꿨다.
# ============================================================

import time

import sensor_input
import fsm_controller
import pressure_balance
import output_controller
import relay_controller
import logger

LOOP_INTERVAL_S = 0.2


def main():
    sensor_reader = sensor_input.SensorReader()
    output = output_controller.OutputController(output_controller.OutputConfig())
    try:
        sensor_reader.init()
        output.init()
        relay_controller.init()
        logger.init()

        print("FLOODGUARD 통합 실행 시작 (실수위 센서·출력·릴레이 사용, IMU 미사용)")

        while True:
            loop_start = time.monotonic()
            data = sensor_reader.read_all()

            state, fsm_reason, sensor_valid = fsm_controller.update(
                h_out_cm=data["h_out_cm"],
                h_in_cm=data["h_in_cm"],
                rise_rate_cm_s=data["rise_rate_cm_s"],
                rise_rate_in_cm_s=data["rise_rate_in_cm_s"],
                roll_deg=data["roll_deg"],
                pitch_deg=data["pitch_deg"],
                imu_valid=data["imu_valid"],
                sonar_valid=data["sonar_valid"],
                severe_tilt=data["severe_tilt"],
            )

            # can_open_flag를 먼저 계산해야 ESCAPE 승격 여부를 정할 수 있다.
            can_open_flag, f_net_n, pressure_reason = pressure_balance.can_open(
                state, data["h_out_cm"], data["h_in_cm"]
            )

            if can_open_flag and state != "ESCAPE":
                state, fsm_reason = fsm_controller.escalate_to_escape()

            output.update_state(state, data["h_out_cm"], data["h_in_cm"])
            relay_result = relay_controller.run(can_open_flag)

            logger.log(
                data, state, fsm_reason, sensor_valid,
                f_net_n, can_open_flag, pressure_reason, relay_result
            )

            print(
                f"state={state} ({fsm_reason}) | h_out={data['h_out_cm']} | "
                f"h_in={data['h_in_cm']} | F_net={f_net_n} | can_open={can_open_flag} "
                f"({pressure_reason}) | relay={relay_result['relay_on']}"
            )

            elapsed = time.monotonic() - loop_start
            remaining = LOOP_INTERVAL_S - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("측정을 종료한다.")

    finally:
        sensor_reader.shutdown()
        output.close()
        relay_controller.close()
        logger.close()


if __name__ == "__main__":
    main()
