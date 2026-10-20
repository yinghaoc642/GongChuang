"""Guard the ring-1 tray-clearance overlay and its no-delay contract."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src/main.cpp").read_text(encoding="utf-8")


def function(name: str) -> str:
    match = re.search(
        r"^[^\n;{}]*\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{",
        SOURCE,
        re.M,
    )
    assert match, name
    index, depth = match.end(), 1
    while depth:
        depth += (SOURCE[index] == "{") - (SOURCE[index] == "}")
        index += 1
    return SOURCE[match.start() : index]


def compact(text: str) -> str:
    return "".join(text.split())


def phase(first: str, after: str) -> str:
    body = function("serviceArmTransfer")
    return body[body.index("case " + first + ":") : body.index("case " + after + ":")]


unload = compact(function("beginUnloadingTransfer"))
reload = compact(function("beginReloadingTransfer"))
release = compact(function("releaseArmTransferDestination"))
deferred = compact(function("serviceArmTransferDeferredStorageCommand"))
restore = compact(function("restoreArmTransferRing1Tray"))
grip_close = compact(
    phase("ARM_TRANSFER_WAIT_GRIP_CLOSE", "ARM_TRANSFER_WAIT_LOADED_CLEARANCE")
)
loaded_clear = compact(
    phase("ARM_TRANSFER_WAIT_LOADED_CLEARANCE", "ARM_TRANSFER_WAIT_DESTINATION_ROTATION")
)
final_clear = compact(
    phase("ARM_TRANSFER_WAIT_FINAL_CLEARANCE", "ARM_TRANSFER_WAIT_FINAL_RETRACT")
)

assert "STORAGE_SERVO_PARK_ANGLE_DEGREES=165.0f;" in compact(SOURCE)
assert (
    "RING1_TRAY_CLEARANCE_ANGLE_DEGREES="
    "STORAGE_SERVO_PARK_ANGLE_DEGREES-180.0f;" in compact(SOURCE)
)
clearance_command = compact(function("commandRing1TrayClearancePosition"))
assert (
    "storageServo.setRawAngleMTurn(RING1_TRAY_CLEARANCE_ANGLE_DEGREES,"
    "STORAGE_SERVO_INTERVAL_MS);" in clearance_command
)

# Scope: both PROCESS rounds and STORAGE round 1 placements. STORAGE round 2
# and RAW cannot select placement-clearance mode 1.
assert (
    "ring1TrayClearanceWorkflow=activeWorkAction==WORK_ACTION_PROCESS||"
    "(activeWorkAction==WORK_ACTION_STORAGE&&workRoundIndex==0U);" in unload
)
assert (
    "ring1TrayClearanceWorkflow&&ringPosition==1U?1U:"
    "(prepareRing1PickupClearance?3U:0U);" in unload
)
assert "secondStorageColorWorkflowActive" not in unload.split(
    "constboolring1TrayClearanceWorkflow", 1
)[1].split("beginArmTransfer", 1)[0]

# Tray-to-ring1: the tray must remain at its pickup slot through gripper close
# and the complete loaded M7 ascent. Parking starts only after lift completion,
# immediately before the existing loaded M5/M6 move.
assert "commandRing1TrayClearancePosition();" not in grip_close
assert loaded_clear.index("if(liftMoveFinished()){") < loaded_clear.index(
    "armTransferRing1TrayClearanceMode==1U"
) < loaded_clear.index("commandRing1TrayClearancePosition();") < loaded_clear.index(
    "startArmTransferPlanarMove("
)

# Ring1-to-tray: parking is commanded before beginArmTransfer can lower M7.
assert (
    "armTransferRing1TrayClearanceMode=ringPosition==1U?2U:"
    "(prepareRing1PickupClearance?3U:0U);" in reload
)
assert reload.index("commandRing1TrayClearancePosition();") < reload.index(
    "beginArmTransfer("
)

# After a ring1 pickup, restore the destination slot only after M7 clearance,
# in the same pass that starts the loaded planar move.
assert loaded_clear.index("if(liftMoveFinished()){") < loaded_clear.index(
    "armTransferRing1TrayClearanceMode==2U"
) < loaded_clear.index("restoreArmTransferRing1Tray();") < loaded_clear.index(
    "startArmTransferPlanarMove("
)

# After a ring1 placement, restore only after M7 clearance. If the next pickup
# is ring1, park after the current transfer is vertically clear and overlap the
# command with the existing M5/M6 preposition interval.
assert final_clear.index("if(liftMoveFinished()){") < final_clear.index(
    "armTransferRing1TrayClearanceMode==1U"
) < final_clear.index("restoreArmTransferRing1Tray();") < final_clear.index(
    "startArmTransferPlanarMove("
)
assert final_clear.index("armTransferRing1TrayClearanceMode==3U") < final_clear.index(
    "commandRing1TrayClearancePosition();"
) < final_clear.index("startArmTransferPlanarMove(")

# The restore command targets the original/next intended slot and retains the
# existing prepared-source handoff when both refer to that same slot.
assert restore.index("commandStorageServoPosition(") < restore.index(
    "armTransferRing1TrayClearanceMode=0U;"
)
assert "armTransferNextStorageCommanded=true;" in restore

# Modes 1 and 3 suppress the ordinary early slot command. Their deadlines are
# derived only from existing release/servo deadlines; no extra delay is added.
assert "armTransferRing1TrayClearanceMode!=1U&&" in release
assert "armTransferRing1TrayClearanceMode!=3U" in release
assert "armTransferRing1TrayClearanceMode==1U||" in release
assert "armTransferRing1TrayClearanceMode==3U" in release
assert (
    "ringReturnSlotDeferred=armTransferUsesReturnProfile()&&"
    "armTransferPrepareNextSource&&armTransferRing1TrayClearanceMode!=3U;"
    in deferred
)
assert (
    "armTransferUsesReturnProfile()?armTransferStorageCommandDueMs+"
    "STORAGE_SERVO_SETTLE_MS:gripperOpenCommandMs+STORAGE_SERVO_SETTLE_MS;"
    in release
)

# Next-ring1 detection is based on the real QR mapped ring positions for both
# the first return pickup and later return pickups.
assert "prepareRing1PickupClearance=firstRingPosition==1U;" in unload
assert "prepareRing1PickupClearance=nextRingPosition==1U;" in reload

for body in (unload, reload, release, deferred, grip_close, loaded_clear, final_clear):
    assert "delay(" not in body
assert "ARM_TRANSFER_WAIT_RING1_TRAY" not in SOURCE

# RAW explicitly clears the overlay; transfer completion clears stale state.
raw = compact(function("beginRawPickupTransfer"))
complete = compact(function("consumeArmTransferCompletion"))
assert "armTransferRing1TrayClearanceMode=0U;" in raw
assert "armTransferRing1TrayRestoreSlot=-1;" in raw
assert "armTransferRing1TrayClearanceMode=0U;" in complete
assert "armTransferRing1TrayRestoreSlot=-1;" in complete

print(
    "PASS ring-1 tray clearance: PROCESS x2 + STORAGE1 only, departure+180 "
    "parking/restoration ordered around M7 clearance with no added wait phase"
)
