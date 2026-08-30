import serial_sender


STAGE_MESSAGES = {
    "IDLE": "SAFE",
    "LOW": "WATER DETECTED",
    "MID": "WARNING",
    "HIGH": "DANGER",
    "ESCAPE": "ESCAPE NOW",
}


def init(dry_run: bool = True) -> None:
    if dry_run:
        print("[DRY RUN] LCD setup skipped")


def update_state(state: str, h_out_cm: float | None, h_in_cm: float | None, dry_run: bool = True) -> bool:
    message = STAGE_MESSAGES.get(state, "UNKNOWN")
    sent = False
    if dry_run:
        print(f"[DRY RUN] LCD {state}: {message} h_out={h_out_cm} h_in={h_in_cm}")
    try:
        sender = serial_sender.SerialStateSender(serial_sender.SerialConfig(dry_run=dry_run))
        sender.connect()
        sent = sender.send_state(state)
        sender.close()
    except Exception as exc:
        if not dry_run:
            raise
        print(f"[DRY RUN] serial skipped: {exc}")
    return sent
