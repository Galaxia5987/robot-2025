package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_RED
import frc.robot.extender
import frc.robot.leds
import frc.robot.lib.distanceFromPoint
import frc.robot.lib.moveBack
import frc.robot.subsystems.alignL2
import frc.robot.subsystems.alignL4
import frc.robot.subsystems.l1
import frc.robot.subsystems.l3
import frc.robot.subsystems.leds.alignPattern
import frc.robot.subsystems.leds.blueTeamPattern
import frc.robot.subsystems.leds.redTeamPattern
import frc.robot.subsystems.outtakeCoral
import frc.robot.subsystems.outtakeCoralAndDriveBack
import frc.robot.subsystems.outtakeL1
import frc.robot.subsystems.raiseElevatorAtDistance
import frc.robot.swerveDrive
import org.littletonrobotics.junction.Logger

var isAligning: Trigger = Trigger { alignCommand().isScheduled }

private fun alignToPose(targetPose: Pose2d, endTrigger: Trigger): Command {
    return swerveDrive
        .runOnce {
            isAligning = Trigger { true }
            leds.setPattern(all = alignPattern).schedule()
            resetProfiledPID(
                swerveDrive.localEstimatedPose,
                -swerveDrive.fieldOrientedSpeeds
            )
            setGoal(targetPose.moveBack(Units.Meters.of(0.1)))
        }
        .andThen(
            swerveDrive
                .run {
                    swerveDrive.limitlessRunVelocity(
                        getSpeed(swerveDrive.localEstimatedPose).invoke()
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
        .finallyDo(Runnable {
            isAligning = Trigger { false }
            leds.setPattern(all = if (IS_RED) redTeamPattern else blueTeamPattern)
                .schedule()
        })
}

fun alignCommand(): Command =
    swerveDrive.defer { alignToPose(selectedScorePose.invoke(), atGoal) }

private fun alignL4Prep(): Command =
    swerveDrive
        .defer {
            alignToPose(
                selectedScorePose.invoke().moveBack(Units.Meters.of(0.3)),
                Trigger { false }
            )
        }
        .raceWith(raiseElevatorAtDistance(alignL4()))

fun alignScoreL1(): Command =
    alignCommand()
        .alongWith(raiseElevatorAtDistance(l1()))
        .andThen(outtakeL1())

fun alignScoreL2(): Command =
    alignCommand()
        .alongWith(raiseElevatorAtDistance(alignL2()))
        .andThen(outtakeCoralAndDriveBack(false))

fun alignScoreL3(): Command =
    alignCommand()
        .alongWith(raiseElevatorAtDistance(l3()))
        .andThen(outtakeCoralAndDriveBack(true))

fun alignScoreL4(): Command =
    alignL4Prep()
        .andThen(alignCommand())
        .andThen(outtakeCoralAndDriveBack(true))

fun autoScoreL4(): Command = alignCommand().andThen(outtakeCoral())

private val atAlignmentSetpoint = Trigger {
    atGoal.asBoolean && isAligning.asBoolean
}

private val isWithinDistance = Trigger {
    swerveDrive.localEstimatedPose.distanceFromPoint(
        selectedScorePose.invoke().translation
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
