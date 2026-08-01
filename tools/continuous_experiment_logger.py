#!/usr/bin/env python3
"""Continuously collect repeated STM32 calibration runs into one log.

The logger survives controller power cycles and serial disconnects.  It adds
PC timestamps and run boundaries while preserving every firmware line.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports


BOOT_MARKER = "[ZERO SAFETY] Before this power-up"
SUCCESS_MARKER = "Rough-processing calibration complete:"
FAILURE_MARKERS = ("FAULT: ", "Controller NOT READY:")


def timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def available_ports() -> list:
    return sorted(list_ports.comports(), key=lambda item: item.device)


def choose_port(requested: str, previous: str | None) -> str | None:
    ports = available_ports()
    devices = {item.device for item in ports}
    if requested.lower() != "auto":
        return requested if requested in devices else None
    if previous in devices:
        return previous
    if len(ports) == 1:
        return ports[0].device

    preferred = [
        item.device
        for item in ports
        if any(
            token in (item.description or "").lower()
            for token in ("usb serial", "ch340", "cp210", "ftdi")
        )
    ]
    return preferred[0] if len(preferred) == 1 else None


class CombinedLog:
    def __init__(self, path: Path, target_runs: int) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("a", encoding="utf-8", buffering=1)
        self.target_runs = target_runs
        self.run_index = 0
        self.terminal_runs = 0
        self.run_open = False
        self.run_terminal = False
        self.boot_seen = False
        self.meta(
            "SESSION START "
            f"target_runs={target_runs} baud=115200 output={path}"
        )

    def close(self) -> None:
        self.meta(
            "SESSION END "
            f"runs_started={self.run_index} terminal_runs={self.terminal_runs}"
        )
        self.stream.close()

    def meta(self, message: str) -> None:
        line = f"[{timestamp()}][LOGGER] {message}"
        self.stream.write(line + "\n")
        print(line, flush=True)

    def begin_run(self, reason: str, saw_boot: bool) -> None:
        if self.run_open and not self.run_terminal:
            self.meta(f"RUN {self.run_index:02d} INCOMPLETE: new boot detected")
            self.terminal_runs += 1
        self.run_index += 1
        self.run_open = True
        self.run_terminal = False
        self.boot_seen = saw_boot
        self.meta(f"========== RUN {self.run_index:02d} BEGIN ({reason}) ==========")

    def serial_line(self, line: str) -> bool:
        if BOOT_MARKER in line:
            if not self.run_open or self.boot_seen:
                self.begin_run("firmware boot", saw_boot=True)
            else:
                self.boot_seen = True
        elif not self.run_open:
            self.begin_run("logger attached after boot", saw_boot=False)

        rendered = (
            f"[{timestamp()}][RUN {self.run_index:02d}][SERIAL] {line}"
        )
        self.stream.write(rendered + "\n")
        print(rendered, flush=True)

        terminal = None
        if SUCCESS_MARKER in line:
            terminal = "SUCCESS"
        elif any(marker in line for marker in FAILURE_MARKERS):
            terminal = "FAILED"
        if terminal is not None and not self.run_terminal:
            self.run_terminal = True
            self.terminal_runs += 1
            self.meta(
                f"========== RUN {self.run_index:02d} {terminal} =========="
            )
        return self.terminal_runs >= self.target_runs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect repeated calibration runs into one timestamped log."
    )
    parser.add_argument(
        "--port",
        default="auto",
        help="COM port such as COM5, or 'auto' when only one adapter is present.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument(
        "--output",
        type=Path,
        help="Combined log path; default is experiment_logs/rough_repeat_<time>.log",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.runs <= 0:
        raise ValueError("--runs must be positive")
    root = Path(__file__).resolve().parents[1]
    output = args.output
    if output is None:
        name = datetime.now().strftime("rough_repeat_%Y%m%d_%H%M%S.log")
        output = root / "experiment_logs" / name
    elif not output.is_absolute():
        output = root / output
    output = output.resolve()
    latest = root / "experiment_logs" / "LATEST.txt"
    latest.parent.mkdir(parents=True, exist_ok=True)
    latest.write_text(str(output), encoding="utf-8")

    log = CombinedLog(output, args.runs)
    selected_port: str | None = None
    last_wait_message = 0.0
    try:
        while log.terminal_runs < args.runs:
            port = choose_port(args.port, selected_port)
            if port is None:
                now = time.monotonic()
                if now - last_wait_message >= 5.0:
                    choices = ", ".join(
                        f"{item.device} ({item.description})"
                        for item in available_ports()
                    ) or "none"
                    log.meta(
                        "WAITING FOR SERIAL PORT; available=" + choices
                    )
                    last_wait_message = now
                time.sleep(0.5)
                continue

            selected_port = port
            try:
                with serial.Serial(
                    port=port,
                    baudrate=args.baud,
                    timeout=0.25,
                    write_timeout=0.25,
                ) as connection:
                    connection.dtr = False
                    connection.rts = False
                    log.meta(f"SERIAL CONNECTED port={port} baud={args.baud}")
                    while log.terminal_runs < args.runs:
                        raw = connection.readline()
                        if not raw:
                            continue
                        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                        if log.serial_line(line):
                            break
            except (serial.SerialException, OSError) as error:
                log.meta(f"SERIAL DISCONNECTED port={port} reason={error}")
                time.sleep(0.5)
        log.meta(f"TARGET REACHED: {args.runs} terminal runs collected")
        return 0
    except KeyboardInterrupt:
        log.meta("STOPPED BY USER (Ctrl+C)")
        return 130
    finally:
        log.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"LOGGER ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
