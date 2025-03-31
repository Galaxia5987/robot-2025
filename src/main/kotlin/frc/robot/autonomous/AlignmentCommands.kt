package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_RED
import frc.robot.RobotContainer
import frc.robot.RobotContainer.driverController
import frc.robot.elevator
import frc.robot.extender
import frc.robot.gripper
import frc.robot.leds
import frc.robot.lib.distanceFromPoint
import frc.robot.lib.flipIfNeeded
import frc.robot.lib.moveBack
import frc.robot.lib.moveTowards
import frc.robot.subsystems.alignmentSetpointL4
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.TunerConstants
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l2algaePickup
import frc.robot.subsystems.l3
import frc.robot.subsystems.l3algaePickup
import frc.robot.subsystems.leds.alignPattern
import frc.robot.subsystems.leds.blueTeamPattern
import frc.robot.subsystems.leds.pathFindPattern
import frc.robot.subsystems.leds.redTeamPattern
import frc.robot.subsystems.netAlgae
import frc.robot.subsystems.outtakeCoralAlignment
import frc.robot.subsystems.outtakeCoralL3Alignment
import frc.robot.subsystems.outtakeL1
import frc.robot.subsystems.outtakeL2
import frc.robot.subsystems.raiseElevatorAtDistance
import frc.robot.swerveDrive
import frc.robot.wrist
import org.littletonrobotics.junction.Logger

var isAligning: Trigger = Trigger { alignCommand().isScheduled }

private fun pathFindToPose(
    pose: Pose2d,
    goalEndVelocity: LinearVelocity = Units.MetersPerSecond.zero()
): Command =
    AutoBuilder.pathfindToPose(
            pose,
            TunerConstants.PATH_CONSTRAINTS,
            goalEndVelocity
        )
        .apply { addRequirements(swerveDrive) }

fun pathFindToSelectedFeeder(): Command =
    swerveDrive
        .defer {
            leds.setPattern(all = pathFindPattern).schedule()
            pathFindToPose(selectedFeeder.invoke())
                .andThen(
                    Commands.run({
                            swerveDrive.limitlessRunVelocity(
                                ChassisSpeeds(0.8, 0.0, 0.0)
                            )
                        })
                        .withTimeout(1.0)
                )
                .finallyDo(
                    Runnable { leds.setLedsBasedOnAlliance().schedule() }
                )
        }
        .until(gripper.hasCoral)

fun getPathfindPoseToScore(): Pose2d {
    if (
        (swerveDrive.pose - selectedScorePose.first.invoke()).translation.x > 0
    ) {
        return selectedScorePose.first.invoke().moveBack(Units.Meters.of(0.5))
    }
    return selectedScorePose.first
        .invoke()
        .moveBack(Units.Meters.of(0.4))
        .moveTowards(swerveDrive.pose, Units.Meters.of(0.4))
}

private fun pathFindToSelectedScorePose(moveBack: Boolean = true): Command {
    return swerveDrive.defer {
        val targetPose =
            if (moveBack) getPathfindPoseToScore()
            else selectedScorePose.first.invoke()

        Logger.recordOutput("pathFindSetpoint", targetPose)
        leds.setPattern(all = pathFindPattern).schedule()
        pathFindToPose(targetPose, PATH_FIND_END_VELOCITY)
            .finallyDo(Runnable { leds.setLedsBasedOnAlliance().schedule() })
    }
}

private fun pathFindToSelectedMiddlePose(): Command {
    return swerveDrive.defer {
        val targetPose =
            selectedScorePose.third.invoke().moveBack(Units.Meters.of(0.5))

        Logger.recordOutput("pathFindSetpoint", targetPose)
        leds.setPattern(all = pathFindPattern).schedule()
        pathFindToPose(targetPose, PATH_FIND_END_VELOCITY)
            .finallyDo(Runnable { leds.setLedsBasedOnAlliance().schedule() })
    }
}

private fun alignToPose(targetPose: Pose2d, endTrigger: Trigger): Command {
    return swerveDrive
        .runOnce {
            isAligning = Trigger { true }
            leds.setPattern(all = alignPattern).schedule()
            resetProfiledPID(
                swerveDrive.localEstimatedPose,
                swerveDrive.localPoseSpeeds
            )
            setGoal(targetPose)
        }
        .andThen(
            swerveDrive
                .run {
                    swerveDrive.fieldOrientedRunVelocity(
                        getSpeed(swerveDrive.localEstimatedPose).invoke(),
                    )
                }
                .alongWith(extender.retractTime(0.3))
        )
        .until(endTrigger)
        .andThen(
            swerveDrive.runOnce {
                swerveDrive.limitlessRunVelocity(ChassisSpeeds())
            }
        )
        .finallyDo(
            Runnable {
                isAligning = Trigger { false }
                leds
                    .setPattern(
                        all = if (IS_RED) redTeamPattern else blueTeamPattern
                    )
                    .schedule()
            }
        )
}

fun alignCommand(moveBack: Boolean = true): Command =
    swerveDrive.defer {
        alignToPose(
            if (moveBack)
                selectedScorePose.first.invoke().moveBack(Units.Meters.of(0.1))
            else selectedScorePose.first.invoke(),
            atGoal
        )
    }

private fun alignToMid(): Command =
    swerveDrive.defer { alignToPose(selectedScorePose.third.invoke(), atGoal) }

private fun alignPrep(reefMasterCommand: Command): Command =
    raiseElevatorAtDistance(reefMasterCommand)
        .raceWith(
            swerveDrive.defer {
                alignToPose(
                    selectedScorePose.first
                        .invoke()
                        .moveBack(Units.Meters.of(0.3)),
                    Trigger { false }
                )
            }
        )

private fun alignPrepToMid(reefMasterCommand: Command): Command =
    reefMasterCommand.withDeadline(
        swerveDrive.defer {
            alignToPose(
                selectedScorePose.third.invoke().moveBack(Units.Meters.of(0.3)),
                elevator.atSetpoint.and(wrist.atSetpoint)
            )
        }
    )

private fun alignPrepToAlgae(reefMasterCommand: Command): Command =
    reefMasterCommand.withDeadline(
        swerveDrive.defer {
            alignToPose(
                selectedScorePose.third.invoke().moveBack(Units.Meters.of(0.3)),
                elevator.atSetpoint.and(wrist.atSetpoint).and(atGoal)
            )
        }
    )

fun alignScoreL1(): Command =
    Commands.sequence(
        pathFindToSelectedMiddlePose()
            .onlyIf(RobotContainer.disablePathFinding.negate()),
        alignPrepToMid(Commands.none()),
        alignToMid().alongWith(l1()),
        outtakeL1()
    )

fun alignScoreL2(): Command =
    Commands.sequence(
        pathFindToSelectedScorePose()
            .onlyIf(RobotContainer.disablePathFinding.negate()),
        alignPrep(Commands.none()),
        alignCommand(false).alongWith(l2()),
        outtakeL2()
    )

fun alignScoreL3(): Command =
    Commands.sequence(
        pathFindToSelectedScorePose()
            .onlyIf(RobotContainer.disablePathFinding.negate()),
        wrist.skyward(),
        alignCommand().alongWith(raiseElevatorAtDistance(l3())),
        outtakeCoralL3Alignment()
    )

fun alignScoreL4(): Command =
    Commands.sequence(
            pathFindToSelectedScorePose()
                .onlyIf(RobotContainer.disablePathFinding.negate()),
            wrist.skyward(),
            (alignCommand()
                .alongWith(raiseElevatorAtDistance(alignmentSetpointL4()))),
            outtakeCoralAlignment(false)
        )
        .withName("alignScoreL4")

fun autoScoreL4(): Command =
    Commands.sequence(
            pathFindToSelectedScorePose()
                .onlyIf(RobotContainer.disablePathFinding.negate()),
            (alignCommand()
                .alongWith(raiseElevatorAtDistance(alignmentSetpointL4()))),
            outtakeCoralAlignment(false)
        )
        .withName("alignScoreL4")

fun alignToReefAlgae2(): Command =
    Commands.sequence(
        pathFindToSelectedMiddlePose()
            .onlyIf(RobotContainer.disablePathFinding.negate()),
        alignPrepToAlgae(l2algaePickup()),
        alignToMid().withDeadline(l2algaePickup()),
        Commands.run({
                swerveDrive.robotOrientedRunVelocity(
                    ChassisSpeeds(-0.5, 0.0, 0.0)
                )
            })
            .withTimeout(0.5),
        wrist.max()
    )

fun alignToReefAlgae3(): Command =
    Commands.sequence(
        pathFindToSelectedMiddlePose()
            .onlyIf(RobotContainer.disablePathFinding.negate()),
        alignPrepToAlgae(l3algaePickup()),
        alignToMid().withDeadline(l3algaePickup()),
        Commands.run({
                swerveDrive.robotOrientedRunVelocity(
                    ChassisSpeeds(-0.5, 0.0, 0.0)
                )
            })
            .withTimeout(0.5),
        wrist.max()
    )

fun alignAlgaeToNet(): Command {
    var isOnOtherSide: Boolean
    return swerveDrive.defer {
        isOnOtherSide = swerveDrive.pose.x < FlippingUtil.fieldSizeX / 2
        val driveAngle = {
            Rotation2d.fromDegrees(if (isOnOtherSide) 180.0 else 0.0)
        }
        val drivePower = { if (isOnOtherSide) 0.7 else -0.7 }
        Logger.recordOutput("AutoAlignment/isOnOtherSide", isOnOtherSide)
        Commands.sequence(
            elevator.zero(),
            wrist.max(),
            DriveCommands.joystickDriveAtAngle(
                    swerveDrive,
                    drivePower,
                    { 0.0 },
                    driveAngle
                )
                .until(ableToNet),
            netAlgae(Trigger { true })
                .alongWith(
                    DriveCommands.joystickDriveAtAngle(
                        swerveDrive,
                        { drivePower.invoke() / 2 },
                        { 0.0 },
                        driveAngle
                    )
                )
        )
    }
}

private val atAlignmentSetpoint = Trigger {
    atGoal.asBoolean && isAligning.asBoolean
}

private val isWithinDistance = Trigger {
    swerveDrive.localEstimatedPose.distanceFromPoint(
        selectedScorePose.first.invoke().translation
    ) in ALIGNMENT_ELEVATOR_MIN_DISTANCE..ALIGNMENT_ELEVATOR_MAX_DISTANCE
}

val shouldOpenElevator = Trigger {
    isWithinDistance.asBoolean && isAligning.asBoolean
}

val ableToNet = Trigger {
    NET_ZONES.any { it.flipIfNeeded().contains(swerveDrive.pose.translation) }
}

val RADIUS_LOOKAHEAD_TIME = Seconds.of(0.5)

fun getPoseLookaheadTime(): Pose2d {
    val transform =
        Transform2d(
            swerveDrive.fieldOrientedSpeeds.vxMetersPerSecond *
                RADIUS_LOOKAHEAD_TIME.`in`(Seconds),
            swerveDrive.fieldOrientedSpeeds.vyMetersPerSecond *
                RADIUS_LOOKAHEAD_TIME.`in`(Seconds),
            Rotation2d.kZero
        )

    if (IS_RED) {
        transform.times(-1.0)
    }

    return swerveDrive.pose.plus(transform)
}

private val isInRadiusOfReef = Trigger {
    getPoseLookaheadTime().distanceFromPoint(ReefCenter.flipIfNeeded()) <
        MOVE_WRIST_UP_RADIUS
}

private val isOutOfReef = Trigger {
    getPoseLookaheadTime().distanceFromPoint(ReefCenter.flipIfNeeded()) >
        MOVE_WRIST_DOWN_RADIUS
}

private val wristCurrentCommandIsNull = Trigger { wrist.currentCommand == null }

val justDidL2: Trigger =
    driverController.square().debounce(1.0, Debouncer.DebounceType.kFalling)

private val doingL1: Trigger =
    driverController.cross().debounce(1.0, Debouncer.DebounceType.kFalling)

private val shouldMoveWristUp =
    (gripper.hasCoral
            .and(isInRadiusOfReef)
            .and(gripper.hasAlgae.negate())
            .and(justDidL2.negate())
            .and(doingL1.negate())
            .and(wristCurrentCommandIsNull)
            .and(CommandGenericHID(3).button(12).negate()))
        .and(RobotModeTriggers.teleop())
        .onTrue(wrist.skyward())

private val shouldCloseWrist =
    gripper.hasAlgae
        .negate()
        .and(isOutOfReef)
        .and(gripper.hasAlgae.negate())
        .and(wristCurrentCommandIsNull)
        .and(CommandGenericHID(3).button(12).negate())
        .and(RobotModeTriggers.teleop())
        .onTrue(wrist.feeder().alongWith(elevator.feeder()))

var isL4 = Trigger { false }

fun logTriggers() {
    mapOf(
            "IsAligning" to isAligning,
            "AtAlignmentSetpoint" to atAlignmentSetpoint,
            "IsWithinDistance" to isWithinDistance,
            "ShouldOpenElevator" to shouldOpenElevator,
            "IsInRadiusOfReef" to isInRadiusOfReef,
            "wristCurrentCommandIsNull" to wristCurrentCommandIsNull,
            "ableToNet" to ableToNet,
            "isL4" to isL4,
            "justDidL2" to justDidL2
        )
        .forEach { (key, value) ->
            Logger.recordOutput("AutoAlignment/$key", value)
        }
}
