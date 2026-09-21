"""FLOODGUARD test 1: integrated sensor, FSM, Arduino, LCD, and relay run.

This runner uses the real water-level sensors, Arduino serial output, LCD,
relay, and merged CSV logger. LOW opens immediately; MID/HIGH follow the
pressure gate in pressure_balance.py.
"""

import time

import fsm_controller
import logger
import output_controller
import pressure_balance
import relay_controller
import sensor_input


LOOP_INTERVAL_S = 0.2


def main():
    sensor_reader = sensor_input.SensorReader(use_imu=False)
    output = output_controller.OutputController(
        output_controller.OutputConfig()
    )
    sensor_ready = False
    output_ready = False
    relay_ready = False
    logger_ready = False

    try:
        sensor_ready = True
        sensor_reader.init()
        output_ready = True
        output.init()
        relay_ready = True
        relay_controller.init()
        logger_ready = True
        logger.init()

        print("FLOODGUARD test1 시작 (실센서·Arduino·LCD·릴레이)")

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
            can_open, f_net_n, pressure_reason = pressure_balance.can_open(
                state, data["h_out_cm"], data["h_in_cm"]
            )

            if can_open and state != "ESCAPE":
                # 개방 진행 문구를 먼저 표시하고, 릴레이 구동 뒤 ESCAPE로 전환한다.
                output.show_opening(state)
                relay_result = relay_controller.run(can_open)
                state, fsm_reason = fsm_controller.escalate_to_escape()
                output.update_state(state, data["h_out_cm"], data["h_in_cm"])
            else:
                output.update_state(state, data["h_out_cm"], data["h_in_cm"])
                relay_result = relay_controller.run(can_open)

            logger.log(
                data,
                state,
                fsm_reason,
                sensor_valid,
                f_net_n,
                can_open,
                pressure_reason,
                relay_result,
            )

            f_net_display = (
                f"{f_net_n:.2f}N" if f_net_n is not None else "N/A"
            )
            print(
                f"state={state} ({fsm_reason}) | h_out={data['h_out_cm']} | "
                f"h_in={data['h_in_cm']} | valid={sensor_valid} | "
                f"F_net={f_net_display} | can_open={can_open} ({pressure_reason}) | "
                f"relay={relay_result['relay_on']} ({relay_result['reason']})"
            )

            remaining = LOOP_INTERVAL_S - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("FLOODGUARD test1 종료 요청")
    finally:
        cleanup = []
        if logger_ready:
            cleanup.append(("CSV", logger.close))
        if relay_ready:
            cleanup.append(("릴레이", relay_controller.close))
        if output_ready:
            cleanup.append(("Arduino IDLE 전송", lambda: output.update_state("IDLE")))
            cleanup.append(("출력", output.close))
        if sensor_ready:
            cleanup.append(("센서", sensor_reader.shutdown))
        for label, close in cleanup:
            try:
                close()
            except Exception as exc:
                print(f"[정리 오류] {label}: {exc}")


if __name__ == "__main__":
    main()
