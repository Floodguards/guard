# ============================================================
# sensor_input.py
#
# 2026-08-25 전면 재작성: 유나(A)가 자신의 작업일지에 "8.25" 페이지로
# 완전히 새로운 센서 입력 설계를 올렸다("FLOODGUARD 센서 입력 및
# 데이터 연동 설계"). 이전 버전("8.16 코드 고도화" 기준으로 병합)과
# 비교해서 가장 큰 변화 두 가지:
#
# !! 2026-08-25 정정 (서연 지적) !! 처음 반영할 때 외부/내부 센서
# 배정을 반대로 적었다. 8.25 문서 "3. 센서 구성" 표 + "HC-SR04P는
# 방수가 아니므로 내부 수면 위에 설치하고... A02YYUW는 방수형이지만
# 커넥터와 Raspberry Pi는 물에 잠기지 않도록 외부 수면보다 높은
# 위치에 고정한다" 원문을 다시 확인한 결과, 올바른 배정은 다음과
# 같다:
#   외부 수위 → A02YYUW (방수형, UART, /dev/serial0)
#   내부 수위 → HC-SR04P (비방수, GPIO TRIG=23/ECHO=24)
# (방수 센서를 물에 노출되는 외부에, 비방수 센서를 보호 덮개를 씌운
# 내부에 두는 게 맞다 - 처음엔 이걸 거꾸로 적었었다.)
#
# !! 변경 1: 외부/내부 센서가 서로 다른 물리 센서로 분리됨 !!
# 이전 버전: A02YYUW(방수 초음파, UART) 하나만 있었고 "외부"만
# 측정했다. 내부(h_in_cm)는 estimate_h_in_cm()이 항상 None을
# 반환하는 미구현 상태였다 - 이게 병합 코드 전체에서 계속 "가장
# 급한 문제"로 표시되어 있던 항목이다.
# 새 버전: 외부는 A02YYUW(방수, UART, /dev/serial0)로, 내부는
# HC-SR04P(비방수, GPIO TRIG/ECHO)로 분리했다. A02YYUW는 이전
# 버전과 마찬가지로 계속 "외부용"이고, 새로 추가된 HC-SR04P가
# "내부용"을 맡는다 - h_in_cm이 이제 실제로 측정된다. 이 부분은
# A의 담당 영역이고 B/C 코드와 충돌이 없어서 확인 즉시 반영했다.
#
# !! 변경 2: roll/pitch를 분리해서 계산함 !!
# 이전 버전은 tilt_deg 하나만 계산해서 roll_deg/pitch_deg 둘 다에
# 똑같이 넣는 임시방편이었다(불일치 보고서 4번 참고, A에게 "IMU에서
# roll/pitch를 분리해서 계산하는지 확인 필요"라고 요청해뒀던 항목).
# 새 버전은 가속도 벡터에서 roll/pitch를 각각 계산한다. 서연 확인
# (2026-08-25): "유나가 롤/피치 분리했으니 이걸로 따라간다"고 결정함.
#
# !! 수위 임계값 관련 !!
# LOW·MID·HIGH 수위 기준은 수조 실험 CSV를 바탕으로 결정한다.
#
# !! 변경 3 (2026-08-25 다섯 번째 갱신): 침수감지 디지털 센서/rollover
# 로직 완전 삭제 !!
# 이전 "8.16" 버전부터 있던 침수감지 디지털 센서(GPIO27, DO 출력)
# 기반 rollover_detected(전복 판정) 로직을 삭제했다. 서연이 원본
# 문서 페이지에 "이 센서가
# 필요할까요??"라고 남긴 코멘트가 있었고, 실제 code_A.zip의
# config.py에도 이 센서 관련 값이 전혀 없는 걸 확인해서 - 유나가
# 이미 뺀 것으로 보고 병합 코드에서도 완전히 삭제하기로 함. 전복
# IMU의 roll/pitch와 severe_tilt는 보정·품질 확인·로그용으로만 남긴다.
# 수위 FSM은 IMU 기울기나 severe_tilt만으로 상태를 바꾸지 않는다.
#
# !! 변경 4 (2026-08-25 여섯 번째 갱신): 8.25 문서와 대조해서 빠져
# 있던 SensorData 필드 2개 추가 !!
# 서연이 "유나 8.25 문서가 다 병합됐는지" 확인을 요청해서 문서 14절
# SensorData 구조와 병합 코드를 항목별로 대조했다. 두 가지가
# 빠져 있었다:
#  1) level_difference_cm (내·외부 수위차, 문서 1번 담당범위 항목
#     "내·외부 수위차 계산") - h_out_cm - h_in_cm 로 추가함.
#  2) outside_valid / inside_valid (문서 14절 SensorData에 sonar_valid와
#     별개로 나열되어 있었으나, 문서 어디에도 정확한 정의가 없었음 -
#     가장 합리적인 해석으로 "해당 센서의 이번 측정이 성공했는지"
#     (raw 값이 None이 아닌지)로 구현함. 문서 원문에 정의가 없는
#     부분이라 유나에게 이 해석이 맞는지 확인 필요 (README_불일치
#     보고서.md 참고).
#
# !! 변경 5 (2026-08-25 일곱 번째 갱신): 함수 방식 -> SensorReader
# 클래스 방식으로 리팩터링 !!
# 서연이 지적함: 실제 code_A.zip의 sensor_input.py는 처음부터
# SensorReader 클래스(메서드 방식)로 짜여 있었는데(문서 17절 main.py
# 예시도 `from sensor_input import SensorReader` + 인스턴스 생성 후
# 메서드 호출), 이 병합 코드는 지금까지 모듈 전역 상태 + 낱개 함수
# 방식이었다 - 8.25 문서의 pseudocode(낱개 함수 설명)를 그대로 따라간
# 결과였는데, 실제 코드 스타일과는 달랐다. 상태(시리얼/센서 객체,
# 필터 버퍼, 캘리브레이션 값)를 인스턴스 속성으로, 상태를 쓰는 함수를
# 메서드로 옮겼다 - 동작은 이전과 동일하다(순수 계산 함수인
# calculate_level/moving_average/calculate_rise_rate/
# vector_to_roll_pitch_deg는 상태가 없어서 모듈 함수로 그대로 뒀다).
# 메서드 이름은 read_all()/init()/shutdown() 등 기존 이름을 그대로
# 유지했다 (문서의 read_sensor_data()라는 이름과는 다름 - main.py와
# fsm_controller.py 쪽 호출부 변경을 최소화하기 위한 선택).
# 부수 효과: 이전에는 모듈을 import하는 순간 SMBus(1)을 실제로 열어서
# 하드웨어에 접근했는데(테스트 시 항상 스텁 필요), 이제는
# SensorReader()를 생성하는 시점으로 미뤄졌다 - 더 안전한 방식이다.
# main.py도 sensor_reader = sensor_input.SensorReader() 인스턴스를
# 만들어서 쓰도록 함께 바꿨다 (main.py 상단 주석 참고).
# ============================================================

import json
import math
import os
import time
from collections import deque

try:
    import serial
except ImportError:
    serial = None

try:
    from gpiozero import DistanceSensor
except ImportError:
    DistanceSensor = None

try:
    from smbus import SMBus
except ImportError:
    SMBus = None


# ----- 이동평균/추세 윈도우 -----
DISTANCE_FILTER_SIZE = 5
TREND_WINDOW_SIZE = 15
RISING_SPEED_THRESHOLD_CM_S = 0.3  # TODO(잠정치): 유나 8.25 문서 기준, 수조 실험 후 조정

# ----- IMU 필터 (보정·로그용) -----
ACCEL_LPF_ALPHA = 0.15

# ----- A02YYUW (외부 수위, UART) -----
# 배선(유나 8.25 문서 4.2): VCC->Pi 3.3V(물리핀17), GND->물리핀20,
# TX->GPIO15/RXD(물리핀10), RX 연결 안 함. 방수형이라 물에 잠기는
# 외부 수면을 측정하는 데 쓰고, 커넥터/Pi 본체는 외부 수면보다
# 높은 위치에 고정한다 (8.25 문서 3절).
OUTSIDE_SERIAL_PORT = "/dev/serial0"
OUTSIDE_SERIAL_BAUDRATE = 9600

# ----- HC-SR04P (내부 수위, GPIO TRIG/ECHO) -----
# 배선(현재 사용자가 연결한 레벨 시프터 구성 기준): 센서 VCC->Pi 5V
# (물리핀 2 또는 4), 센서 GND/Pi GND/시프터 GND 공통.
# 시프터 HV->Pi 5V, LV->Pi 3.3V.
# Pi GPIO23(물리핀16)->LV1, HV1->센서 TRIG;
# 센서 ECHO->HV2, LV2->Pi GPIO24(물리핀18).
# GPIO 핀 번호와 gpiozero 설정은 변하지 않는다.
# 비방수 센서라서 내부 수면 위쪽에 설치하고, 물이 직접 튀지 않도록
# 보호 덮개를 씌운다 (덮개가 초음파 송수신부를 막으면 안 됨).
# gpiozero.DistanceSensor가 트리거 펄스 발사 + 에코 시간 측정을
# 내부에서 처리해준다 (직접 GPIO 타이밍 코드를 짤 필요 없음).
HC_SR04P_TRIGGER_GPIO = 23
HC_SR04P_ECHO_GPIO = 24
HC_SR04P_MAX_DISTANCE_M = 0.5  # 30cm 모형 기준 여유치

MPU6050_ADDRESS = 0x68
MPU6050_ACCEL_REGISTER = 0x3B

# ----- 캘리브레이션 파일 (유나 8.25 문서 7.2) -----
# 실행한 현재 디렉터리가 달라도 floodguard_b 안의 동일한 파일을
# 모든 테스트와 main.py가 함께 사용한다.
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(PROJECT_DIR, "calibration", "sensor_calibration.json")
DEFAULT_CALIBRATION = {
    "outside_base_distance_cm": 30.0,   # TODO: 실측 후 교체 (init()에서 캘리브레이션 시 갱신)
    "inside_base_distance_cm": 30.0,    # TODO: 실측 후 교체
    "baseline_roll_deg": 0.0,
    "baseline_pitch_deg": 0.0,
}


# ============================================================
# 순수 계산 함수 (인스턴스 상태 없음 - 모듈 함수로 유지)
# ============================================================

def calculate_level(base_distance_cm, measured_distance_cm):
    """유나 8.25 문서 8절 calculate_level() 그대로."""
    if measured_distance_cm is None:
        return None
    level_cm = base_distance_cm - measured_distance_cm
    return max(0.0, min(30.0, level_cm))


def moving_average(buffer, new_value):
    buffer.append(new_value)
    return sum(buffer) / len(buffer)


def calculate_rise_rate(history):
    """유나 8.25 문서 10절 calculate_rise_rate() 그대로 (선형회귀 기울기)."""
    if len(history) < 5:
        return 0.0

    base_time = history[0][0]
    times = [t - base_time for t, _ in history]
    levels = [lv for _, lv in history]

    average_time = sum(times) / len(times)
    average_level = sum(levels) / len(levels)

    numerator = sum((t - average_time) * (lv - average_level) for t, lv in zip(times, levels))
    denominator = sum((t - average_time) ** 2 for t in times)

    if denominator == 0:
        return 0.0

    return numerator / denominator


def vector_to_roll_pitch_deg(accel_vector):
    ax, ay, az = accel_vector
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))
    return roll, pitch


def _convert_signed_16bit(high_byte, low_byte):
    value = (high_byte << 8) | low_byte
    if value >= 0x8000:
        value -= 65536
    return value


# ============================================================
# SensorReader - 센서 상태를 들고 있는 클래스 (2026-08-25 일곱
# 번째 갱신: 위 헤더 "변경 5" 참고, 실제 code_A.zip 구조를 따름)
# ============================================================

class SensorReader:
    def __init__(self, use_imu=False):
        self.use_imu = use_imu
        self.outside_serial = None  # serial.Serial (A02YYUW, 외부), init()에서 생성
        self._outside_uart_buffer = bytearray()
        self.inside_sensor = None   # gpiozero.DistanceSensor (HC-SR04P, 내부), init()에서 생성
        self.i2c_bus = None

        if self.use_imu:
            if SMBus is None:
                raise RuntimeError("smbus is required for real sensor input")
            self.i2c_bus = SMBus(1)
            self.i2c_bus.write_byte_data(MPU6050_ADDRESS, 0x6B, 0x00)
            time.sleep(0.1)

        self.outside_distance_buffer = deque(maxlen=DISTANCE_FILTER_SIZE)
        self.inside_distance_buffer = deque(maxlen=DISTANCE_FILTER_SIZE)
        self.outside_trend_buffer = deque(maxlen=TREND_WINDOW_SIZE)
        self.inside_trend_buffer = deque(maxlen=TREND_WINDOW_SIZE)

        self.filtered_accel_vector = None
        self._calibration = dict(DEFAULT_CALIBRATION)

    # ----- 캘리브레이션 (유나 8.25 문서 7절) -----

    def load_calibration(self):
        if os.path.exists(CALIBRATION_FILE):
            with open(CALIBRATION_FILE, "r") as f:
                self._calibration = json.load(f)
        else:
            self._calibration = dict(DEFAULT_CALIBRATION)
        return self._calibration

    def save_calibration(self):
        os.makedirs(os.path.dirname(CALIBRATION_FILE), exist_ok=True)
        with open(CALIBRATION_FILE, "w") as f:
            json.dump(self._calibration, f, indent=2)

    def calibrate_empty_tank(self, sample_count=30):
        """수조가 빈 상태에서 호출. 외부(A02YYUW)/내부(HC-SR04P) 거리
        기준값 + IMU 기준 roll/pitch를 저장한다 (유나 8.25 문서 7.1
        캘리브레이션 방법)."""
        outside_samples = []
        inside_samples = []
        for _ in range(sample_count):
            d_out = self.read_a02yyuw_distance_cm()
            if d_out is not None:
                outside_samples.append(d_out)
            d_in = self.read_hc_sr04p_distance_cm()
            if d_in is not None:
                inside_samples.append(d_in)
            time.sleep(0.05)

        if self.use_imu:
            baseline_vector = self.calibrate_initial_orientation()
            baseline_roll, baseline_pitch = vector_to_roll_pitch_deg(baseline_vector)
        else:
            # 수압 실험 등 IMU를 제외한 단계에서는 중립 자세를 기준으로 둔다.
            baseline_roll = baseline_pitch = 0.0

        minimum_samples = max(1, sample_count // 2)
        if len(outside_samples) < minimum_samples:
            raise RuntimeError("외부 A02YYUW 캘리브레이션 측정값이 부족하다.")
        if len(inside_samples) < minimum_samples:
            raise RuntimeError("내부 HC-SR04P 캘리브레이션 측정값이 부족하다.")

        self._calibration = {
            "outside_base_distance_cm": sum(outside_samples) / len(outside_samples),
            "inside_base_distance_cm": sum(inside_samples) / len(inside_samples),
            "baseline_roll_deg": baseline_roll,
            "baseline_pitch_deg": baseline_pitch,
        }
        self.save_calibration()
        return self._calibration

    # ----- A02YYUW (외부 수위) -----

    def read_a02yyuw_distance_cm(self):
        """Drain queued UART frames and return only the newest valid distance."""
        if self.outside_serial is None:
            return None

        deadline = time.monotonic() + 0.5
        latest_distance_cm = None

        while time.monotonic() < deadline:
            waiting = self.outside_serial.in_waiting
            chunk = self.outside_serial.read(waiting if waiting else 1)
            if not chunk:
                break

            self._outside_uart_buffer.extend(chunk)

            while True:
                header_index = self._outside_uart_buffer.find(b"\xff")
                if header_index < 0:
                    self._outside_uart_buffer.clear()
                    break
                if header_index > 0:
                    del self._outside_uart_buffer[:header_index]
                if len(self._outside_uart_buffer) < 4:
                    break

                frame = self._outside_uart_buffer[:4]
                calculated_checksum = (frame[0] + frame[1] + frame[2]) & 0xFF
                if frame[3] == calculated_checksum:
                    distance_mm = (frame[1] << 8) + frame[2]
                    if 30 <= distance_mm <= 4500:
                        latest_distance_cm = distance_mm / 10.0
                    del self._outside_uart_buffer[:4]
                else:
                    del self._outside_uart_buffer[0]

            if latest_distance_cm is not None and self.outside_serial.in_waiting == 0:
                return latest_distance_cm

        return latest_distance_cm

    # ----- HC-SR04P (내부 수위) -----

    def read_hc_sr04p_distance_cm(self):
        if self.inside_sensor is None:
            return None
        try:
            distance_m = self.inside_sensor.distance
        except Exception:
            return None
        if distance_m is None:
            return None
        return distance_m * 100.0

    # ----- MPU6050 / roll·pitch 분리 계산 -----
    #
    # !! 2026-08-25 변경 !! 이전 버전은 기준 중력벡터와의 각도차이
    # 하나(tilt_deg)만 계산해서 roll/pitch에 똑같이 넣었다. 유나가
    # roll/pitch를 분리했다고 확인해서, 표준 가속도계 tilt 공식으로
    # roll/pitch를 각각 계산하도록 바꿨다:
    #   roll  = atan2(ay, az)
    #   pitch = atan2(-ax, sqrt(ay^2 + az^2))
    # 계산된 값에서 캘리브레이션 시점의 baseline_roll/pitch를 빼서
    # "상대 기울기"를 구한다 (유나 8.25 문서 11절과 동일한 방식).

    def read_mpu6050(self):
        data = self.i2c_bus.read_i2c_block_data(MPU6050_ADDRESS, MPU6050_ACCEL_REGISTER, 14)
        accel = tuple(_convert_signed_16bit(data[i], data[i + 1]) / 16384.0 for i in (0, 2, 4))
        gyro = tuple(_convert_signed_16bit(data[i], data[i + 1]) / 131.0 for i in (8, 10, 12))
        return {"accel": accel, "gyro": gyro}

    def apply_accelerometer_low_pass(self, acceleration):
        if self.filtered_accel_vector is None:
            self.filtered_accel_vector = acceleration
            return self.filtered_accel_vector
        alpha = ACCEL_LPF_ALPHA
        self.filtered_accel_vector = tuple(
            alpha * acceleration[i] + (1 - alpha) * self.filtered_accel_vector[i]
            for i in range(3)
        )
        return self.filtered_accel_vector

    def calibrate_initial_orientation(self, sample_count=100):
        accel_sum = [0.0, 0.0, 0.0]
        print("MPU6050 기준 자세를 보정한다. 차량 모형을 움직이지 않는다.")
        for _ in range(sample_count):
            acceleration = self.read_mpu6050()["accel"]
            for i in range(3):
                accel_sum[i] += acceleration[i]
            time.sleep(0.01)
        average_vector = tuple(v / sample_count for v in accel_sum)
        print("MPU6050 기준 자세 보정을 완료하였다.")
        return average_vector

    # ----- 초기화 / 메인 루프 -----

    def init(self):
        if serial is None or DistanceSensor is None:
            raise RuntimeError("pyserial and gpiozero are required for real sensor input")

        self.outside_serial = serial.Serial(
            port=OUTSIDE_SERIAL_PORT,
            baudrate=OUTSIDE_SERIAL_BAUDRATE,
            timeout=0.3,
        )
        self.outside_serial.reset_input_buffer()
        self.inside_sensor = DistanceSensor(
            echo=HC_SR04P_ECHO_GPIO,
            trigger=HC_SR04P_TRIGGER_GPIO,
            max_distance=HC_SR04P_MAX_DISTANCE_M,
        )

        self.load_calibration()

    def read_all(self):
        loop_time = time.monotonic()

        # ----- 수위 (외부: A02YYUW, 내부: HC-SR04P) -----
        try:
            outside_raw_cm = self.read_a02yyuw_distance_cm()
        except Exception:
            outside_raw_cm = None
        try:
            inside_raw_cm = self.read_hc_sr04p_distance_cm()
        except Exception:
            inside_raw_cm = None

        outside_filtered_cm = (
            moving_average(self.outside_distance_buffer, outside_raw_cm)
            if outside_raw_cm is not None else None
        )
        inside_filtered_cm = (
            moving_average(self.inside_distance_buffer, inside_raw_cm)
            if inside_raw_cm is not None else None
        )

        h_out_cm = calculate_level(self._calibration["outside_base_distance_cm"], outside_filtered_cm)
        h_in_cm = calculate_level(self._calibration["inside_base_distance_cm"], inside_filtered_cm)

        if h_out_cm is not None:
            self.outside_trend_buffer.append((loop_time, h_out_cm))
        if h_in_cm is not None:
            # 상승률 표본은 FSM 상태(LOW/MID 등)와 무관하게 첫 측정부터 쌓는다.
            # 다섯 표본이 모이면 calculate_rise_rate()가 기울기를 계산한다.
            self.inside_trend_buffer.append((loop_time, h_in_cm))

        rise_rate_out_cm_s = calculate_rise_rate(list(self.outside_trend_buffer))
        rise_rate_in_cm_s = calculate_rise_rate(list(self.inside_trend_buffer))

        # ----- 기울기 (roll/pitch 분리) -----
        if self.use_imu:
            try:
                mpu_data = self.read_mpu6050()
                filtered_acceleration = self.apply_accelerometer_low_pass(mpu_data["accel"])
                measured_roll_deg, measured_pitch_deg = vector_to_roll_pitch_deg(filtered_acceleration)
                roll_deg = measured_roll_deg - self._calibration["baseline_roll_deg"]
                pitch_deg = measured_pitch_deg - self._calibration["baseline_pitch_deg"]
                imu_valid = True
            except Exception:
                roll_deg = pitch_deg = 0.0
                imu_valid = False
        else:
            # MPU6050을 아직 사용하지 않는 실험 단계에서는 수평으로 가정한다.
            roll_deg = pitch_deg = 0.0
            imu_valid = True

        outside_valid = outside_raw_cm is not None
        inside_valid = inside_raw_cm is not None
        # 초음파 센서의 측정 성공 여부만 수위 FSM의 센서 유효성으로 사용한다.
        # IMU 상태와 기울기는 선택적 보정·로그용이며 수위 판정을 막지 않는다.
        sonar_valid = outside_valid and inside_valid
        # 기울기 임계값 판정은 사용하지 않는다. 기존 CSV 열과 호출부
        # 호환성을 위해 severe_tilt는 항상 False로 기록한다.
        severe_tilt = False

        # 2026-08-25 여섯 번째 갱신: 문서 14절 SensorData와 대조해서 추가
        # (위 헤더 "변경 4" 참고).
        level_difference_cm = (
            h_out_cm - h_in_cm if h_out_cm is not None and h_in_cm is not None else None
        )
        return {
            "loop_time": loop_time,
            "outside_raw_distance_cm": outside_raw_cm,
            "outside_distance_cm": outside_filtered_cm,
            "inside_raw_distance_cm": inside_raw_cm,
            "inside_distance_cm": inside_filtered_cm,
            "h_out_cm": h_out_cm,
            "h_in_cm": h_in_cm,
            "level_difference_cm": level_difference_cm,
            "rise_rate_out_cm_s": rise_rate_out_cm_s,
            "rise_rate_in_cm_s": rise_rate_in_cm_s,
            # main.py/fsm_controller 호환용 별칭 (예전 이름 유지)
            "rise_rate_cm_s": rise_rate_out_cm_s,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "outside_valid": outside_valid,
            "inside_valid": inside_valid,
            "sonar_valid": sonar_valid,
            "severe_tilt": severe_tilt,
            "imu_valid": imu_valid,
        }

    def shutdown(self):
        if self.outside_serial is not None:
            self.outside_serial.close()
        if self.inside_sensor is not None:
            self.inside_sensor.close()
        if self.i2c_bus is not None:
            self.i2c_bus.close()
