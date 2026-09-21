# ============================================================
# pressure_balance.py
# 2026-08-25 historical note: 당시 정연(B)의 실제 실행 로그에서
# f_net_n 계산을 확인했으며, 그때의 37cm 사각판/폭 가정은 아래
# 2026-09-21 trapezoid remeasurement update로 대체됐다.
#
# 기존 F_net ≤ 57.7N은 μ=0.83 가정에 따른 계산 추정치였다.
# 2026-09-21 재측정: 패널은 사다리꼴이며 아래폭 33.3cm, 위폭 36.4cm,
# 높이 21.7cm(기존 측정과 동일), 두께 3mm다. 수압 폭은 수위에 따라
# 선형으로 변한다고 보고 패널 높이 방향으로 압력을 적분한다.
# 판 밀도 0.55g/cm³, 구동력 50.4N, μ_s=0.50은 아직 설계 추정값이다.
# 새 치수 기준 판 부피 약 226.87cm³, 추정 질량 약 0.1248kg,
# (50.4 - 0.1248×9.8)/0.50 ≈ 98.35N. 구조 실측 전 계산 추정치다.
# μ_k=0.30으로 구한 약 163.9N은 운동 시작 뒤 참고값일 뿐 개방 기준이 아니다.
#
# 2026-08-25 historical model (superseded 2026-09-21): rectangular width
# 0.37m and PRESSURE_THRESHOLD_N=108.2N. Current dimensions/model are the
# trapezoid constants and integrated pressure expression below.
# Negative net pressure remains clamped to zero, per the original policy.
# 2) "알 수 없는 상태: 열지 않음"이 개방 판단표에 명시되어 있어서,
#    should_open_window()에 IDLE/LOW/MID/HIGH/ESCAPE가 아닌 상태가
#    들어오면 명시적으로 False를 반환하도록 unknown-state 분기를
#    추가함.
#
# 2026-08-30 최종 통합 기준: ESCAPE가 fsm_controller의 정식 FSM
# 상태로 유지되며, should_open_window()도 ESCAPE를 명시적으로
# 처리한다. ESCAPE는 창문이 이미 열린 뒤의 상태이므로 can_open=False를
# 반환한다. can_open은 "지금 릴레이를 작동할지"를 뜻하므로, ESCAPE에서
# 릴레이를 다시 작동시키지 않는다.
# ============================================================

RHO_WATER = 1000.0
G = 9.8

# 사다리꼴 패널 실측값. 위/아래는 판을 세웠을 때의 수평 폭이다.
PANEL_HEIGHT_M = 0.217
PANEL_BOTTOM_WIDTH_M = 0.333
PANEL_TOP_WIDTH_M = 0.364
PANEL_THICKNESS_M = 0.003
PANEL_DENSITY_KG_M3 = 550.0  # 0.55g/cm³ 가정, 실측 전
PANEL_AREA_M2 = PANEL_HEIGHT_M * (PANEL_BOTTOM_WIDTH_M + PANEL_TOP_WIDTH_M) / 2.0
PANEL_VOLUME_M3 = PANEL_AREA_M2 * PANEL_THICKNESS_M
PANEL_MASS_KG = PANEL_VOLUME_M3 * PANEL_DENSITY_KG_M3

# 설계 추정 입력값: μ_s=0.50, μ_k=0.30. 실제 접촉 조합·젖은 조건의 실측값은 아니다.
# 최대 선형 구동력 50.4N과 새 패널 무게를 적용한 정지마찰 기준 추정치.
MAX_LINEAR_DRIVE_FORCE_N = 50.4
STATIC_FRICTION_COEFF = 0.50   # PVC 경질판 자료 범위를 참고한 설계 추정값
KINETIC_FRICTION_COEFF = 0.30  # PVC 접촉 실험값을 참고한 설계 추정값
PRESSURE_THRESHOLD_N = (
    MAX_LINEAR_DRIVE_FORCE_N - PANEL_MASS_KG * G
) / STATIC_FRICTION_COEFF  # 약 98.35N, 실제 구조 및 실험 검증 필요


def compute_f_net_n(h_out_cm, h_in_cm):
    """수위에 따라 폭이 선형으로 변하는 사다리꼴 패널의 순수압력(N).

    한 면의 압력 합력은 rho*g*∫(h-z)w(z)dz이며 z는 판 바닥부터의
    높이다. 수위가 판 상단을 넘어도 판 전체의 압력을 적분한다.
    안쪽 합력이 바깥쪽보다 크면 기존 정책대로 순힘을 0으로 클램프한다.
    """
    h_out_m = (h_out_cm or 0.0) / 100.0
    h_in_m = (h_in_cm or 0.0) / 100.0

    def one_side_force_n(water_height_m):
        wetted_height_m = min(max(water_height_m, 0.0), PANEL_HEIGHT_M)
        bottom_width = PANEL_BOTTOM_WIDTH_M
        width_change = PANEL_TOP_WIDTH_M - PANEL_BOTTOM_WIDTH_M
        pressure_integral = (
            bottom_width
            * (water_height_m * wetted_height_m - wetted_height_m ** 2 / 2.0)
            + (width_change / PANEL_HEIGHT_M)
            * (
                water_height_m * wetted_height_m ** 2 / 2.0
                - wetted_height_m ** 3 / 3.0
            )
        )
        return RHO_WATER * G * pressure_integral

    diff = one_side_force_n(h_out_m) - one_side_force_n(h_in_m)
    if diff < 0:
        diff = 0.0
    return diff


def should_open_window(state, h_out_cm, h_in_cm):
    f_net_n = compute_f_net_n(h_out_cm, h_in_cm)

    if state == "IDLE":
        return False, f_net_n, "idle_no_open"

    if state == "LOW":
        return True, f_net_n, "low_immediate_open"

    if state == "ESCAPE":
        # 창문이 이미 열린 뒤의 상태. can_open은 릴레이 재작동을 막기 위해 False.
        return False, f_net_n, "escape_already_open"

    if state not in ("MID", "HIGH"):
        # "알 수 없는 상태: 열지 않음" (정연의 개방판단 문서 기준,
        # 2026-08-25 추가) - IDLE/LOW/MID/HIGH/ESCAPE가 아닌 상태가
        # 실수로 들어와도 안전하게 열지 않는다.
        return False, f_net_n, "unknown_state_no_open"

    if h_in_cm is None:
        return False, f_net_n, "h_in_unavailable_wait"

    if f_net_n <= PRESSURE_THRESHOLD_N:
        return True, f_net_n, "pressure_balanced_open"

    return False, f_net_n, "pressure_too_high_wait"


can_open = should_open_window
