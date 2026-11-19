#!/usr/bin/env python3

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def compact_cpp(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    source = re.sub(r"//[^\n]*", "", source)
    return re.sub(r"\s+", "", source)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    source = compact_cpp(
        (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
    )

    require(
        "ARM_AXIS_POSITION_ACK_TIMEOUT_MS=100UL;" in source
        and "ARM_AXIS_POSITION_ACK_STATUS_GRACE_MS=60UL;" in source,
        "bounded position-ACK deadline is missing",
    )
    require(
        "!axis.commandAcknowledged&&!axis.motionObserved" in source
        and "postCommandStatusSeen||statusGraceExpired" in source
        and "ARM_AXIS_RECOVERY_ACK_TIMEOUT" in source,
        "missing ACK is not separated from a command that actually moved",
    )
    require(
        "requestKind==ARM_LINEAR_REQUEST_STATUS||"
        "requestKind==ARM_LINEAR_REQUEST_POSITION_QUERY" in source
        and "axis->lastRequestKind=ARM_LINEAR_REQUEST_NONE;" in source,
        "query-related 00 EE still faults an accepted motion",
    )
    require(
        "requestKind==ARM_LINEAR_REQUEST_POSITION&&"
        "!axis->commandAcknowledged&&!axis->motionObserved" in source
        and "ARM_AXIS_RECOVERY_COMMAND_REJECTED" in source,
        "a position-command 00 EE does not enter bounded recovery",
    )
    require(
        "axis.address==liftAxis.address?M7_PROFILE_USE_REQUESTED:"
        "axis.commandM7ProfilePolicy" in source,
        "M7 recovery still reapplies the experimental multiplier",
    )

    print(
        "PASS EMM command recovery: missing ACK is recovered early, "
        "query 00 EE is nonfatal, and M7 retry stays at 2160/171"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError) as error:
        print(f"EMM COMMAND RECOVERY CHECK FAILED: {error}")
        raise SystemExit(1)
