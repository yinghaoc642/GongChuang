#!/usr/bin/env python3

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.cpp"
CONFIG = (
    ROOT / "lib" / "RobotConfig" / "src" / "RobotConfig.h"
)
PLANNER = (
    ROOT
    / "lib"
    / "ArmTransferPlanner"
    / "src"
    / "ArmTransferPlanner.cpp"
)
PLANNER_HEADER = (
    ROOT
    / "lib"
    / "ArmTransferPlanner"
    / "src"
    / "ArmTransferPlanner.h"
)

def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)

def compact(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"\s+", "", text)

def main() -> int:
    source = MAIN.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")
    planner_source = PLANNER.read_text(encoding="utf-8")
    planner_header = PLANNER_HEADER.read_text(encoding="utf-8")
    code = compact(source)
    tuning = compact(config)
    planner = compact(planner_source)

    require(
        "M7_TRAVEL_SPEED_RPM=6740U;" in tuning
        and "M7_TRAVEL_ACCELERATION=239U;" in tuning
        and "M7_ZERO_SOFT_LANDING_DISTANCE_MM=2.0f;" in tuning
        and "M7_ZERO_SOFT_LANDING_SPEED_RPM=2160U;" in tuning
        and "M7_CONTACT_SOFT_LANDING_DISTANCE_MM=2.0f;"
        in tuning
        and "M6_CONTACT_SOFT_LANDING_DISTANCE_MM=2.0f;"
        in tuning,
        "M7 fast-travel/zero cushion or contact soft segments changed",
    )
    require(
        "M7_ZERO_SOFT_LANDING_ACCELERATION=171U;" in tuning
        and "M7_CONTACT_SOFT_LANDING_ACCELERATION=171U;"
        in tuning
        and "M6_CONTACT_SOFT_LANDING_ACCELERATION=149U;"
        in tuning,
        "soft-segment accelerations are not physically increased by 50%",
    )

    require(
        "m6ContactSoftLandingPending=true;" in code
        and "m6ContactSoftLandingTargetMm=extensionMm;" in code
        and "extensionMm-directionSign*"
        "M6_CONTACT_SOFT_LANDING_DISTANCE_MM" in code,
        "M6 two-stage contact approach is missing",
    )
    require(
        "!m6ContactSoftLandingPending;" in code,
        "M6 may report completion between the fast and soft segments",
    )
    require(
        "m7SoftLandingIsContact=true;" in code
        and "m7SoftLandingTargetMm=heightMm;" in code
        and "heightMm+M7_CONTACT_SOFT_LANDING_DISTANCE_MM"
        in code,
        "M7 contact descent is not split 2 mm before the target",
    )
    require(
        "speedRpm<M7_ZERO_SOFT_LANDING_SPEED_RPM"
        "?speedRpm:M7_ZERO_SOFT_LANDING_SPEED_RPM;" in code
        and "m7SoftLandingSpeedRpm=finalSpeedRpm;" in code,
        "M7 zero approach does not enforce the 2160 RPM final cap",
    )
    require(
        "!m7SoftLandingPending;" in code,
        "M7 may report completion between the fast and soft segments",
    )

    require(
        code.count(
            "startArmTransferLiftToSourceHeightMm("
            "armTransferSourcePose.heightMm)"
        ) == 2,
        "transfer pickup descents do not use the source-side policy",
    )
    require(
        "startArmTransferLiftToContactHeightMm("
        "armTransferSourcePose.heightMm)" not in code,
        "a pickup descent still uses the destination soft-landing policy",
    )
    require(
        "if(startArmTransferLiftToContactHeightMm("
        "armTransferDestinationPose.heightMm)){"
        "armTransferPhase=ARM_TRANSFER_WAIT_DESTINATION_LOWER;}"
        in code,
        "destination descent can advance phase after a rejected M7 command",
    )
    require(
        "if(!armTransferMapSource){"
        "SerialDebug.println("
        '"[TRAYPICK]directsingle-segmentdescent;nocontactcushion");'
        "returnstartLinearAxisMove("
        "liftAxis,heightMm,M7_MINIMUM_HEIGHT_MM,"
        "M7_STANDARD_HEIGHT_MM,M7_PULSES_PER_MM,"
        "M7_RAISE_DIRECTION,M7_LOWER_DIRECTION,"
        "M7_SPEED_RPM,M7_ACCELERATION);}" in code,
        "tray pickup is not an explicit single direct M7 descent",
    )
    require(
        "if(armTransferUsesRawProfile()){"
        "SerialDebug.println("
        '"[RAWPICK]test-matcheddirectsingle-segmentM7descent");'
        "returnstartLinearAxisMove("
        "liftAxis,heightMm,M7_MINIMUM_HEIGHT_MM,"
        "M7_STANDARD_HEIGHT_MM,M7_PULSES_PER_MM,"
        "M7_RAISE_DIRECTION,M7_LOWER_DIRECTION,"
        "M7_SPEED_RPM,M7_ACCELERATION);}" in code
        and "if(armTransferUsesRawProfile()){"
        "returnstartLiftMoveWithContactSoftLanding("
        "heightMm,RAW_M7_SPEED_RPM,RAW_M7_ACCELERATION);}" not in code,
        "RAW pickup descent does not exactly match the test M7 profile",
    )
    require(
        "if(armTransferUsesRawProfile()){"
        "SerialDebug.println("
        '"[TRAYPLACE]RAWdirectsingle-segmentdescent;nocontactcushion");'
        "returnstartLinearAxisMove("
        "liftAxis,heightMm,M7_MINIMUM_HEIGHT_MM,"
        "M7_STANDARD_HEIGHT_MM,M7_PULSES_PER_MM,"
        "M7_RAISE_DIRECTION,M7_LOWER_DIRECTION,"
        "RAW_M7_SPEED_RPM,RAW_M7_ACCELERATION);}" in code,
        "RAW-to-tray placement still contains a contact-soft segment",
    )
    require(
        "if(startLiftToHeightMmWithProfile("
        "armTransferDestinationPose.heightMm,"
        "M7_RING_PLACE_SPEED_RPM,"
        "M7_RING_PLACE_ACCELERATION)){"
        "armTransferPhase=ARM_TRANSFER_WAIT_DESTINATION_LOWER;}"
        in code
        and "M7_RING_PLACE_SPEED_RPM=2700U;" in tuning
        and "M7_RING_PLACE_ACCELERATION=199U;" in tuning
        and "RING_PLACE_EXTENSION_SETTLE_MS" not in code
        and "RING_PLACE_LOWER_SETTLE_MS" not in code,
        "gentle ring placement did not retain the shortened 2 mm final segment",
    )
    require(
        "startArmTransferExtensionToMm(" not in code
        and "startArmTransferSourceExtensionToMm(" in code
        and "startArmTransferDestinationExtensionToMm(" in code,
        "source and destination M6 policies are not separated",
    )
    require(
        "if(armTransferUsesRawProfile()){"
        "returnstartExtensionToContactMmWithProfile("
        "extensionMm,minimumMm,RAW_M6_SPEED_RPM,"
        "RAW_M6_ACCELERATION);}"
        "if(armTransferUsesReturnProfile()||mappedRingPose){"
        "returnstartLinearAxisMove(" in code,
        "RAW pickup or ring-return M6 source policy changed unexpectedly",
    )
    require(
        "M6_STANDARD_SPEED_RPM=702U;" in tuning
        and "M6_RAW_SPEED_RPM=756U;" in tuning
        and "M6_RETURN_SPEED_RPM=M6_PLACE_SPEED_RPM;" in tuning
        and "M6_RETURN_ACCELERATION=M6_PLACE_ACCELERATION;" in tuning
        and "M6_LOADED_RETURN_SPEED_RPM=M6_PLACE_SPEED_RPM;"
        in tuning
        and "M6_LOADED_RETURN_ACCELERATION=M6_PLACE_ACCELERATION;"
        in tuning
        and "M7_RETURN_SPEED_RPM="
        "arm_hardware::M7_TRAVEL_SPEED_RPM;" in tuning
        and "M7_RETURN_ACCELERATION="
        "arm_hardware::M7_TRAVEL_ACCELERATION;" in tuning
        and "M6_PLACE_SPEED_RPM=810U;" in tuning
        and "M6_PLACE_ACCELERATION=208U;" in tuning,
        "M6/M7 directional transfer profiles are not identical",
    )
    require(
        "if(armTransferUsesReturnProfile()){"
        "SerialDebug.println("
        '"[RINGRETURN]directsingle-segmentsourcedescent");'
        "returnstartLinearAxisMove("
        "liftAxis,heightMm,M7_MINIMUM_HEIGHT_MM,"
        "M7_STANDARD_HEIGHT_MM,M7_PULSES_PER_MM,"
        "M7_RAISE_DIRECTION,M7_LOWER_DIRECTION,"
        "RETURN_M7_SPEED_RPM,RETURN_M7_ACCELERATION);}" in code,
        "ring-return source pickup is not one direct M7 move",
    )
    require(
        "sourceExtensionPolicy?"
        "startArmTransferSourceExtensionToMm("
        "pose.extensionMm,mappedPose):"
        "startArmTransferDestinationExtensionToMm("
        "pose.extensionMm,mappedPose,loadedReturnMotion);" in code,
        "parallel planar helper does not preserve source/destination M6 policy",
    )
    require(
        "M5_STANDARD_MAXIMUM_STEP_RATE=210600.0f;" in tuning
        and "M5_PLACE_MAXIMUM_STEP_RATE=181440.0f;" in tuning
        and "M5_PLACE_STEP_ACCELERATION=72000.0f;" in tuning
        and "M5_RETURN_MAXIMUM_STEP_RATE="
        "M5_PLACE_MAXIMUM_STEP_RATE;" in tuning
        and "M5_RETURN_STEP_ACCELERATION="
        "M5_PLACE_STEP_ACCELERATION;" in tuning
        and "M5_LOADED_RETURN_MAXIMUM_STEP_RATE="
        "M5_PLACE_MAXIMUM_STEP_RATE;"
        in tuning
        and "M5_LOADED_RETURN_STEP_ACCELERATION="
        "M5_PLACE_STEP_ACCELERATION;"
        in tuning
        and "M5_RAW_MAXIMUM_STEP_RATE=210600.0f;" in tuning,
        "M5 directional transfer profiles are not identical",
    )
    require(
        "PROFILE_RING_PLACE" in planner_header
        and "PROFILE_RING_RETURN" in planner_header
        and "selectMotionProfile(" in planner_source
        and "useArmBasePlaceMotionProfile();" in code
        and "useArmBaseReturnMotionProfile();" in code,
        "placement and ring return do not select independent profiles",
    )
    require(
        "beginArmTransfer(source,arm_transfer::containerReturnPlacePose(),"
        "false,true,false,false,sourceAlreadyPrepared,"
        "prepareNextRingPickup?&nextRingPickupPose:nullptr,"
        "prepareNextRingPickup?static_cast<int8_t>(nextStorageSlot):"
        "static_cast<int8_t>(-1),false,false,true,true);" in code,
        "ring pickup must skip the redundant gripper-open settling delay",
    )
    require(
        "if(sourceAlreadyPrepared&&!sourceGripperAlreadyOpen){"
        "routeFault("
        '"Preparedtransfersourcerequiresgripperalreadyopen");'
        "return;}" in code
        and "Transfersourcereadinessmodeconflict" not in code
        and "static_cast<int8_t>(-1),false,false,"
        "sourceAlreadyPrepared||concurrentSourcePreparation,"
        "prepareFirstPlacedRingPickup);" in code,
        "prepared second pickup can still reject its already-open gripper state",
    )
    require(
        "CONTAINER_PLACE_ANGLE_DEGREES=-102.0f;" in tuning
        and "CONTAINER_RETURN_PLACE_ANGLE_DEGREES=-102.0f;"
        in tuning
        and "containerReturnPlacePose()" in planner,
        "ring return M5 destination still contains the removed 5-degree offset",
    )
    require(
        "TRAY_ROTATION_CLEARANCE_HEIGHT_MM=-10.0f;" in tuning
        and "RING_ROTATION_CLEARANCE_HEIGHT_MM=-10.0f;" in tuning
        and "ARM_TRANSFER_WAIT_PREPARE_CLEARANCE" in code
        and "ARM_TRANSFER_WAIT_LOADED_CLEARANCE" in code
        and "ARM_TRANSFER_WAIT_FINAL_CLEARANCE" in code
        and "ARM_TRANSFER_WAIT_LOADED_LIFT" not in code
        and "ARM_TRANSFER_WAIT_FINAL_LIFT" not in code
        and "ARM_TRANSFER_WAIT_SOURCE_EXTENSION" not in code
        and "ARM_TRANSFER_WAIT_DESTINATION_EXTENSION" not in code
        and "ARM_TRANSFER_BASE_SETTLE_MS=5UL;" in code
        and "ARM_TRANSFER_RETURN_SETTLE_MS=0UL;" in code
        and "GRIPPER_INTERVAL_MS=60U;" in code
        and "GRIPPER_TARGET_PLACE_OPEN_INTERVAL_MS=40U;" in code
        and "GRIPPER_DOUBLE_SPEED_INTERVAL_MS=20U;" in code
        and "GRIPPER_OPEN_SETTLE_MS=60UL;" in code
        and "GRIPPER_CLOSE_SETTLE_MS=120UL;" in code
        and "GRIPPER_TARGET_PLACE_OPEN_SETTLE_MS=40UL;" in code
        and "GRIPPER_TRAY_RELEASE_OPEN_SETTLE_MS=20UL;" in code
        and "GRIPPER_TARGET_PICK_CLOSE_SETTLE_MS=120UL;" in code
        and "GRIPPER_TRAY_PICK_CLOSE_SETTLE_MS=160UL;" in code
        and "armTransferTransitionSettleMs()" in code
        and "armTransferGripperOpenSettleMs()" in code
        and "armTransferGripperCloseSettleMs()" in code
        and "armTransferUsesPlaceProfile()||"
        "armTransferUsesReturnProfile()" in code,
        "return transitions still pause or gripper timing was not reduced",
    )
    require(
        "voidcommandGripperDoubleSpeedOpen(){"
        "gripperServo.setRawAngle(GRIPPER_OPEN_ANGLE_DEGREES,"
        "GRIPPER_DOUBLE_SPEED_INTERVAL_MS,GRIPPER_OPEN_POWER_MW);}" in code
        and "voidcommandGripperDoubleSpeedClose(){"
        "gripperServo.setRawAngle(GRIPPER_CLOSE_ANGLE_DEGREES,"
        "GRIPPER_DOUBLE_SPEED_INTERVAL_MS,GRIPPER_CLOSE_POWER_MW);}" in code
        and "WORK_M7_TO_GRIPPER_GAP_MS=10UL;" in code
        and "WORK_GRIPPER_TO_M7_GAP_MS=10UL;" in code
        and "ARM_TRANSFER_WAIT_SOURCE_GRIPPER_GAP" in code
        and "ARM_TRANSFER_WAIT_DESTINATION_GRIPPER_GAP" in code
        and "millis()+armTransferGripperCloseSettleMs()+"
        "WORK_GRIPPER_TO_M7_GAP_MS;" in code
        and "gripperOpenCommandMs+"
        "armTransferGripperOpenSettleMs()+"
        "(armGripperLiftIsolationEnabled?"
        "WORK_GRIPPER_TO_M7_GAP_MS:0UL);" in code
        and "armGripperLiftIsolationEnabled=kind!=WORK_ACTION_NONE;"
        in code,
        "workstation gripper/M7 short non-overlap handoff is missing",
    )
    require(
        "prepareNextRingPickup=workItemIndex<2U;" in code
        and "ringPickupPrepositionedPending" in code
        and "armTransferNextStorageCommanded" in code
        and "otherAxis.active||otherAxis.recoveryPending" in code
        and '"ConcurrentM6/M7commandrejectedonsharedEMMserial"'
        in code
        and "[EMMPARALLEL]" not in code
        and "linearAxisForAddress(frame[0])" in code
        and "armLinearPositionQueryAxis=&axis;" in code
        and "boolstatusRequestIssued=false;" in code,
        "ring-return prepositioning or strict shared-bus exclusion is missing",
    )
    require(
        "caseARM_TRANSFER_WAIT_LOADED_CLEARANCE:"
        "if(liftMoveFinished()){"
        "if(startArmTransferPlanarMove("
        "armTransferDestinationPose,armTransferMapDestination,"
        "armTransferMapSource,false," in code
        and '"loadedclearance->destination",'
        "armTransferUsesReturnProfile())" in code
        and "caseARM_TRANSFER_WAIT_FINAL_CLEARANCE:"
        "if(liftMoveFinished())" in code
        and '"releaseclearance->nextpose"' in code
        and "startArmBaseStandardFrameDegrees("
        "pose.standardFrameAngleDegrees);"
        "rotationStarted=true;constboolextensionStarted=" in code,
        "M5 and M6 are not started together after M7 reaches clearance",
    )
    require(
        "RING_RETURN_STORAGE_COMMAND_DELAY_MS=500UL;" in code
        and "armTransferStorageCommandDueMs=gripperOpenCommandMs+"
        "armTransferGripperOpenSettleMs()+"
        "RING_RETURN_STORAGE_COMMAND_DELAY_MS;" in code
        and "armTransferNextStorageDeadlineMs=millis()+"
        "STORAGE_SERVO_SETTLE_MS;" in code
        and "armTransferNextStorageCommanded=true;" in code
        and "WORK_PHASE_WAIT_STORAGE_SERVO" in code,
        "ring-return storage servo lacks the 500 ms release delay or arrival gate",
    )
    require(
        "if(!liftMoveFinished()||liftAxis.currentMm<"
        "requiredClearanceMm-ARM_AXIS_POSITION_TOLERANCE_MM)"
        in code
        and '"M5/M6planarmoverequestedbelowM7clearance"'
        in code
        and "otherAxis.active||otherAxis.recoveryPending" in code,
        "clearance or shared-serial protection can permit an unsafe move",
    )
    require(
        "ARM_AXIS_EXPECTED_COMPLETION_VERIFY_MARGIN_MS=250UL;"
        in code
        and "axis.timeoutMs-"
        "ARM_AXIS_EXPECTED_COMPLETION_VERIFY_MARGIN_MS" in code
        and "ARM_AXIS_EXPECTED_COMPLETION_VERIFY_MS" not in code,
        "encoder terminal verification can still fire at the old fixed 700 ms",
    )
    require(
        '"[RINGRETURN]directsingle-segmenttraydescent"' in code
        and "RETURN_M7_SPEED_RPM,RETURN_M7_ACCELERATION);}" in code
        and "if(startArmTransferLiftToContactHeightMm("
        "armTransferDestinationPose.heightMm))" in code,
        "ring return still contains a tray soft-landing segment",
    )
    require(
        "CONTAINER_PICK_PHYSICAL_LOWER_MM=41.0f;" in tuning
        and "SECOND_PICK_EXTRA_LOWER_MM=3.0f;" in tuning
        and "placementSequenceIndex==1U?"
        "config::arm_transfer::SECOND_PICK_EXTRA_LOWER_MM:0.0f"
        in planner
        and "arm_transfer::containerPickPose(workItemIndex)" in code,
        "all tray pickups do not lower M7 by 3 mm while retaining the "
        "second-slot compensation",
    )
    require(
        "CONTAINER_PICK_EXTENSION_MM=1.0f;" in tuning,
        "tray pickup M6 extension is not reduced from 3 mm to 1 mm",
    )
    require(
        "RING_RETURN_PICK_EXTRA_LOWER_MM=3.0f;" in tuning
        and code.count(
            "PROCESS_PLACE_LOWER_MM+"
            "arm_config::RING_RETURN_PICK_EXTRA_LOWER_MM"
        ) == 4
        and "[RINGRETURNPICK]extraM7lower/target-mm=" in code,
        "ring-return pickup and next-pick preposition do not use the "
        "extra 3 mm M7 descent",
    )
    require(
        "MAPPED_RING_EXTENSION_REDUCTION_MM=8.0f;" in tuning
        and "correctedExtensionMm=rawExtensionMm-"
        "arm_config::MAPPED_RING_EXTENSION_REDUCTION_MM;" in code
        and "pose.extensionMm=commandedExtensionMm;" in code,
        "visual ring-pose settlement does not retain the 8 mm M6 reduction",
    )
    require(
        "TARGET_PLACE_EXTRA_LOWER_MM=5.0f;" in tuning
        and code.count("applyTargetPlacementExtraLower(destination)") == 1
        and "pose.heightMm=correctedHeightMm;" in code,
        "target placement does not apply exactly one extra 5 mm M7 descent",
    )
    require(
        "M6_RING2_MINIMUM_EXTENSION_MM=-8.0f;" in code
        and "M6_STARTUP_WORKING_ZERO_OFFSET_MM+"
        "M6_RING2_MINIMUM_EXTENSION_MM>=2.0f," in code
        and "Ring-2M6retractionmustretain2mmhard-stopmargin"
        in code,
        "M6 near-side retraction is not limited to -8 mm with a 2 mm "
        "physical hard-stop margin",
    )
    require(
        "pathUsesMappedNearSideRange="
        "mappedRingPose||extensionMm<"
        "M6_STANDARD_EXTENSION_MM-"
        "ARM_AXIS_POSITION_TOLERANCE_MM||"
        "extensionAxis.currentMm<"
        "M6_STANDARD_EXTENSION_MM-"
        "ARM_AXIS_POSITION_TOLERANCE_MM;" in code,
        "M6 soft return from ring 2 may reject its negative approach point",
    )

    print(
        "PASS arm motion policy: M7-to--10 first, M5/M6 planar parallel, "
        "strict M6/M7 exclusion, 5/0 ms transfer settling, direct tray "
        "motion, and guarded prepared-source handoff"
    )
    return 0

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError) as error:
        print(f"ARM SOFT-LANDING CHECK FAILED: {error}")
        raise SystemExit(1)
