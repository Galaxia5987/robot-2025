package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_RED
import frc.robot.RobotContainer
import frc.robot.extender
import frc.robot.leds
import frc.robot.lib.distanceFromPoint
import frc.robot.lib.moveBack
import frc.robot.lib.moveTowards
import frc.robot.subsystems.alignmentSetpointL4
import frc.robot.subsystems.drive.TunerConstants
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l3
import frc.robot.subsystems.leds.alignPattern
import frc.robot.subsystems.leds.blueTeamPattern
import frc.robot.subsystems.leds.pathFindPattern
import frc.robot.subsystems.leds.redTeamPattern
import frc.robot.subsystems.outtakeCoral
import frc.robot.subsystems.outtakeCoralAndDriveBack
import frc.robot.subsystems.outtakeL1
import frc.robot.subsystems.outtakeL2
import frc.robot.subsystems.raiseElevatorAtDistance
import frc.robot.swerveDrive
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

fun pathFindToSelectedFeeder(): Command =
    swerveDrive.defer {
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
                Runnable {
                    leds
                        .setPattern(
                            all =
                                if (IS_RED) redTeamPattern else blueTeamPattern
                        )
                        .schedule()
                }
            )
    }

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
            .finallyDo(
                Runnable {
                    leds
                        .setPattern(
                            all =
                                if (IS_RED) redTeamPattern else blueTeamPattern
                        )
                        .schedule()
                }
            )
    }
}

private fun alignToPose(targetPose: Pose2d, endTrigger: Trigger): Command {
    return swerveDrive
        .runOnce {
            isAligning = Trigger { true }
            leds.setPattern(all = alignPattern).schedule()
            resetProfiledPID(
                swerveDrive.localEstimatedPose,
                swerveDrive.fieldOrientedSpeeds
            )
            setGoal(targetPose)
        }
        .andThen(
            swerveDrive
                .run {
                    swerveDrive.fieldOrientedRunVelocity(
                        getSpeed(swerveDrive.localEstimatedPose).invoke(),
                        IS_RED
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

private fun alignPrep(reefMasterCommand: Command): Command =
    swerveDrive
        .defer {
            alignToPose(
                selectedScorePose.first.invoke().moveBack(Units.Meters.of(0.3)),
                Trigger { false }
            )
        }
        .raceWith(raiseElevatorAtDistance(reefMasterCommand))

fun alignScoreL1(): Command =
    alignCommand(false)
        .alongWith(raiseElevatorAtDistance(l1()))
        .andThen(outtakeL1())

fun alignScoreL2(): Command =
    Commands.sequence(
        pathFindToSelectedScorePose()
            .onlyIf(RobotContainer.disableAlignment.negate()),
        alignPrep(l2()),
        alignCommand(false),
        outtakeL2()
    )

fun alignScoreL3(): Command =
    Commands.sequence(
        pathFindToSelectedScorePose()
            .onlyIf(RobotContainer.disablePathFinding.negate()),
        alignPrep(l3()),
        alignCommand(),
        outtakeCoralAndDriveBack(false)
    )

fun alignScoreL4(): Command =
    Commands.sequence(
        pathFindToSelectedScorePose()
            .onlyIf(RobotContainer.disablePathFinding.negate()),
        alignPrep(alignmentSetpointL4()),
        alignCommand(),
        outtakeCoralAndDriveBack(true)
    )

fun autoScoreL4(): Command = alignCommand().andThen(outtakeCoral())

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

var isL4 = Trigger { false }

fun logTriggers() {
    mapOf(
            "IsAligning" to isAligning,
            "AtAlignmentSetpoint" to atAlignmentSetpoint,
            "IsWithinDistance" to isWithinDistance,
            "ShouldOpenElevator" to shouldOpenElevator,
            "isL4" to isL4
        )
        .forEach { (key, value) ->
            Logger.recordOutput("AutoAlignment/$key", value)
        }
}
