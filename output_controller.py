# ============================================================
# output_controller.py
# C파트의 OutputController 구조를 기준으로,
# B파트의 serial_sender.py와 LCD 출력을 하나의 호출부로 통합한 병합 코드.
# serial_sender.py는 B파트 원본 출처 보존용으로 별도 보관한다.
# ============================================================

from dataclasses import dataclass
import serial_sender


STAGE_MESSAGES = {
    "IDLE": ("시스템 대기중", "정상 상태입니다"),
    "LOW": ("침수 감지됨", "창문 개방 중.."),
    "MID": ("경고! 수위상승", "창문 재시도중.."),
    "HIGH": ("!위험! 수압대기", "균형시 창문개방"),
    "ESCAPE": ("즉시 탈출하세요!", "창문 개방 완료"),
}


@dataclass(frozen=True)
class OutputConfig:
    lcd_i2c_address: int = 0x27
    lcd_cols: int = 16
    lcd_rows: int = 2
    lcd_dotsize: int = 8


class OutputController:
    """Arduino 시리얼 전송과 LCD 갱신을 하나의 출력 인터페이스로 제공한다."""

    def __init__(self, config: OutputConfig | None = None):
        self.config = config or OutputConfig()
        self.sender = serial_sender.SerialStateSender(serial_sender.SerialConfig())
        self.lcd = None
        self.last_displayed_state: str | None = None

    def init(self) -> None:
        self.sender.connect()
        from RPLCD.i2c import CharLCD

        self.lcd = CharLCD(
            i2c_expander="PCF8574",
            address=self.config.lcd_i2c_address,
            cols=self.config.lcd_cols,
            rows=self.config.lcd_rows,
            dotsize=self.config.lcd_dotsize,
        )

    def update_state(
        self,
        state: str,
        h_out_cm: float | None = None,
        h_in_cm: float | None = None,
    ) -> bool:
        """상태를 Arduino에 보내고, 상태가 바뀌었을 때만 LCD를 갱신한다."""
        sent = self.sender.send_state(state)
        if state == self.last_displayed_state:
            return sent

        if state not in STAGE_MESSAGES:
            print(f"[output_controller][WARN] 알 수 없는 상태: {state}")
            return sent

        line1, line2 = STAGE_MESSAGES[state]
        if self.lcd is None:
            raise RuntimeError("LCD가 초기화되지 않았습니다. init()을 먼저 호출하세요.")
        self.lcd.clear()
        self.lcd.write_string(line1[: self.config.lcd_cols])
        self.lcd.crlf()
        self.lcd.write_string(line2[: self.config.lcd_cols])

        self.last_displayed_state = state
        return sent

    def show_opening(self, state: str) -> bool:
        """Show the opening-in-progress message before the relay is energized."""
        if state not in ("LOW", "MID", "HIGH"):
            raise ValueError(f"개방 중 표시를 지원하지 않는 상태: {state}")

        sent = self.sender.send_state(state)
        line1 = STAGE_MESSAGES[state][0]
        if self.lcd is None:
            raise RuntimeError("LCD가 초기화되지 않았습니다. init()을 먼저 호출하세요.")

        self.lcd.clear()
        self.lcd.write_string(line1[: self.config.lcd_cols])
        self.lcd.crlf()
        self.lcd.write_string("창문 개방 중.."[: self.config.lcd_cols])
        self.last_displayed_state = "OPENING"
        return sent

    def close(self) -> None:
        self.sender.close()
        if self.lcd is not None:
            self.lcd.close()
            self.lcd = None
