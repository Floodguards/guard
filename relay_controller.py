from dataclasses import dataclass
import time

try:
    from gpiozero import OutputDevice
except ImportError:
    OutputDevice = None


@dataclass(frozen=True)
class RelayConfig:
    gpio_pin: int = 17  # 라즈베리파이에서 릴레이 IN에 연결할 GPIO 핀
    active_high: bool = True  # 릴레이가 HIGH 신호에서 켜지는지 여부
    max_run_s: float = 3.0  # 모터 최대 구동 시간
    dry_run: bool = True  # 실제 GPIO 없이 테스트하는 모드


@dataclass(frozen=True)
class RelayDecision:
    relay_on: bool  # 릴레이가 실제로 동작했는지
    already_opened: bool  # 이미 창문 개방을 실행했는지
    reason: str  # 판단 이유


# 실제 릴레이 없이 테스트할 때 사용하는 가짜 릴레이
class MockRelay:
    def on(self) -> None:
        print("[DRY RUN] relay ON")

    def off(self) -> None:
        print("[DRY RUN] relay OFF")


# Pi에서 릴레이를 직접 제어하는 클래스
class RelayController:
    def __init__(self, config: RelayConfig | None = None):
        self.config = config or RelayConfig()
        self.already_opened = False
        self.relay = None

    # 릴레이 준비
    def setup(self) -> None:
        if self.config.dry_run:
            self.relay = MockRelay()
            print("[DRY RUN] Relay setup skipped")
            return

        if OutputDevice is None:
            raise RuntimeError("install gpiozero on Raspberry Pi")

        # active_high=True면 HIGH에서 릴레이 ON
        self.relay = OutputDevice(
            self.config.gpio_pin,
            active_high=self.config.active_high,
            initial_value=False,
        )

    # 창문 개방을 한 번만 실행
    def open_once(self, can_open: bool) -> RelayDecision:
        if not can_open:
            return RelayDecision(
                relay_on=False,
                already_opened=self.already_opened,
                reason="open_condition_false",
            )

        if self.already_opened:
            return RelayDecision(
                relay_on=False,
                already_opened=True,
                reason="already_opened_skip",
            )

        if self.relay is None:
            raise RuntimeError("Relay is not ready. Call setup() first.")

        self.relay.on()
        time.sleep(self.config.max_run_s)
        self.relay.off()

        self.already_opened = True
        return RelayDecision(
            relay_on=True,
            already_opened=True,
            reason="relay_open_executed",
        )

    # 릴레이 정리
    def close(self) -> None:
        if self.relay is not None:
            self.relay.off()
            close = getattr(self.relay, "close", None)
            if close is not None:
                close()
            self.relay = None


# 테스트용 함수
def run_demo() -> None:
    controller = RelayController(
        RelayConfig(
            max_run_s=0.5,
            dry_run=True,
        )
    )

    controller.setup()

    samples = [False, True, True]

    for can_open in samples:
        decision = controller.open_once(can_open)
        print(
            f"can_open={can_open} "
            f"relay_on={decision.relay_on} "
            f"already_opened={decision.already_opened} "
            f"reason={decision.reason}"
        )

    controller.close()


if __name__ == "__main__":
    run_demo()
