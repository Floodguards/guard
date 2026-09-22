import time

import sensor_input
import fsm_controller

from RPLCD.i2c import CharLCD


# ============================================================
# 설정
# ============================================================

LOOP_INTERVAL_S = 0.5

# LCD I2C 주소
# i2cdetect 결과가 3F라면 0x3F로 변경
LCD_ADDRESS = 0x27


# ============================================================
# LCD 초기화
# ============================================================

lcd = CharLCD(
    i2c_expander="PCF8574",
    address=LCD_ADDRESS,
    port=1,
    cols=16,
    rows=2,
    dotsize=8,
)


def lcd_show(line1, line2):
    """
    16x2 LCD에 두 줄 표시
    """
    lcd.clear()

    lcd.cursor_pos = (0, 0)
    lcd.write_string(line1[:16])

    lcd.cursor_pos = (1, 0)
    lcd.write_string(line2[:16])


# ============================================================
# MAIN
# ============================================================

def main():

    # IMU는 이번 테스트에서 사용하지 않음
    sensor_reader = sensor_input.SensorReader(use_imu=False)

    try:

        print("=" * 50)
        print("FLOODGUARD Raspberry Pi ONLY TEST")
        print("Arduino / Relay 사용 안 함")
        print("=" * 50)

        # ----------------------------------------------------
        # 센서 초기화
        # ----------------------------------------------------

        sensor_reader.init()

        print("센서 초기화 완료")

        lcd_show(
            "FLOODGUARD",
            "SENSOR TEST"
        )

        time.sleep(2)

        # ----------------------------------------------------
        # 빈 수조 캘리브레이션
        # ----------------------------------------------------

        print()
        print("빈 수조 캘리브레이션을 시작합니다.")
        print("수조에 물이 없는 상태인지 확인하세요.")

        input("준비되었으면 ENTER를 누르세요.")

        calibration = sensor_reader.calibrate_empty_tank(
            sample_count=30
        )

        print()
        print("=== CALIBRATION ===")

        print(
            f"Outside base : "
            f"{calibration['outside_base_distance_cm']:.2f} cm"
        )

        print(
            f"Inside base  : "
            f"{calibration['inside_base_distance_cm']:.2f} cm"
        )

        print("===================")

        lcd_show(
            "CALIBRATION",
            "COMPLETE"
        )

        time.sleep(2)

        # ----------------------------------------------------
        # 반복 측정
        # ----------------------------------------------------

        print()
        print("측정을 시작합니다.")
        print("Ctrl + C 로 종료할 수 있습니다.")
        print()

        while True:

            loop_start = time.monotonic()

            # -----------------------------------------------
            # 1. 초음파 센서 측정
            # -----------------------------------------------

            data = sensor_reader.read_all()

            h_out = data["h_out_cm"]
            h_in = data["h_in_cm"]

            rise_out = data["rise_rate_cm_s"]
            rise_in = data["rise_rate_in_cm_s"]

            # -----------------------------------------------
            # 2. FSM 상태 판단
            # -----------------------------------------------

            state, reason, sensor_valid = fsm_controller.update(
                h_out_cm=h_out,
                h_in_cm=h_in,
                rise_rate_cm_s=rise_out,
                rise_rate_in_cm_s=rise_in,
                roll_deg=data["roll_deg"],
                pitch_deg=data["pitch_deg"],
                imu_valid=data["imu_valid"],
                sonar_valid=data["sonar_valid"],
                severe_tilt=data["severe_tilt"],
            )

            # -----------------------------------------------
            # 3. 터미널 출력
            # -----------------------------------------------

            print(
                f"OUT={h_out} cm | "
                f"IN={h_in} cm | "
                f"Rise={rise_in:.2f} cm/s | "
                f"STATE={state} | "
                f"VALID={sensor_valid}"
            )

            # -----------------------------------------------
            # 4. LCD 출력
            # -----------------------------------------------

            if h_out is None:
                out_text = "OUT: ERROR"
            else:
                out_text = f"OUT:{h_out:5.1f}cm"

            if h_in is None:
                in_text = "IN: ERROR"
            else:
                in_text = f"IN:{h_in:5.1f}cm"

            lcd_show(
                out_text,
                f"{state} {in_text}"
            )

            # -----------------------------------------------
            # 5. 0.5초 간격
            # -----------------------------------------------

            elapsed = time.monotonic() - loop_start

            remaining = LOOP_INTERVAL_S - elapsed

            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:

        print()
        print("측정을 종료합니다.")

    finally:

        sensor_reader.shutdown()

        lcd.clear()
        lcd.close()


# ============================================================
# 실행
# ============================================================

if __name__ == "__main__":
    main()
