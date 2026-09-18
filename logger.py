# ============================================================
# logger.py
# B파트 업로드 logger.py를 기준으로 확인함. A 파트 원시값
# 원시값 컬럼은 병합 과정에서 추가·연결했으며, B 원본과
# 병합용 확장 내용을 구분한다.
#
# 2026-08-25 갱신: 유나의 새 sensor_input.py(8.25 설계, 외부 A02YYUW
# /내부 HC-SR04P 분리 + roll/pitch 분리)에 맞춰 컬럼을 갱신했다.
# 옛 컬럼(raw_distance_cm/corrected_distance_cm/tilt_deg/sonar_reliable)은
# outside_raw_distance_cm/outside_distance_cm/inside_raw_distance_cm/
# inside_distance_cm/roll_deg/pitch_deg/sonar_valid/severe_tilt로 대체됨.
#
# 2026-08-25 다섯 번째 갱신: 침수감지 디지털 센서/rollover 로직을
# 완전히 삭제하면서 outside_water_detected, rollover_detected 두
# 컬럼도 같이 제거했다 (sensor_input.py 상단 주석 참고).
#
# 2026-08-25 여섯 번째 갱신: 유나 8.25 문서 15절 CSV 저장 형식과
# 대조해서 빠져 있던 level_difference_cm, outside_valid, inside_valid
# 컬럼을 추가했다 (sensor_input.py 상단 "변경 4" 주석 참고).
# ============================================================

import csv
import os
from datetime import datetime

CSV_FILE_NAME = "floodguard_merged_log.csv"

_HEADER = [
    "timestamp_s",
    # B 파트 (정연 원본 컬럼)
    "state", "fsm_reason", "sensor_valid",
    "f_net_n", "can_open", "pressure_reason",
    "relay_on", "relay_reason",
    # A 파트 원시값 (2026-08-25, 유나 8.25 설계 반영)
    "outside_raw_distance_cm", "outside_distance_cm",
    "inside_raw_distance_cm", "inside_distance_cm",
    "h_out_cm", "h_in_cm", "level_difference_cm",
    "rise_rate_out_cm_s", "rise_rate_in_cm_s",
    "roll_deg", "pitch_deg",
    "outside_valid", "inside_valid", "sonar_valid", "severe_tilt",
]

_file = None
_writer = None


def _value_or_blank(sensor_data, key):
    value = sensor_data.get(key)
    return "" if value is None else value


def init():
    global _file, _writer
    write_header = not os.path.exists(CSV_FILE_NAME) or os.path.getsize(CSV_FILE_NAME) == 0
    _file = open(CSV_FILE_NAME, "a", newline="")
    _writer = csv.writer(_file)
    if write_header:
        _writer.writerow(_HEADER)


def log(sensor_data, fsm_state, fsm_reason, sensor_valid,
        f_net_n, can_open_flag, pressure_reason, relay_result):
    _writer.writerow([
        datetime.now().isoformat(timespec="milliseconds"),
        fsm_state, fsm_reason, int(sensor_valid),
        round(f_net_n, 2) if f_net_n is not None else "",
        int(can_open_flag), pressure_reason,
        int(relay_result["relay_on"]), relay_result["reason"],
        _value_or_blank(sensor_data, "outside_raw_distance_cm"),
        _value_or_blank(sensor_data, "outside_distance_cm"),
        _value_or_blank(sensor_data, "inside_raw_distance_cm"),
        _value_or_blank(sensor_data, "inside_distance_cm"),
        _value_or_blank(sensor_data, "h_out_cm"),
        _value_or_blank(sensor_data, "h_in_cm"),
        _value_or_blank(sensor_data, "level_difference_cm"),
        round(sensor_data.get("rise_rate_out_cm_s") or 0, 3),
        round(sensor_data.get("rise_rate_in_cm_s") or 0, 3),
        round(sensor_data.get("roll_deg") or 0, 2),
        round(sensor_data.get("pitch_deg") or 0, 2),
        int(sensor_data.get("outside_valid") or False),
        int(sensor_data.get("inside_valid") or False),
        int(sensor_data.get("sonar_valid") or False),
        int(sensor_data.get("severe_tilt") or False),
    ])
    _file.flush()


def close():
    if _file:
        _file.close()
