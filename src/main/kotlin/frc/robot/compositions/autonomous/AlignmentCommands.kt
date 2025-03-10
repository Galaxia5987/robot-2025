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
import frc.robot.lib.extensions.cm
import frc.robot.lib.moveBack
import frc.robot.subsystems.alignmentSetpointL4
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
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
            setGoal(targetPose)
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
                selectedScorePose.first.invoke().moveBack(10.cm)
            else selectedScorePose.first.invoke(),
            atGoal
        )
    }

private fun alignPrep(reefMasterCommand: Command): Command =
    swerveDrive
        .defer {
            alignToPose(
                selectedScorePose.first.invoke().moveBack(30.cm),
                Trigger { false }
            )
        }
        .raceWith(raiseElevatorAtDistance(reefMasterCommand))

fun alignScoreL1(): Command =
    alignCommand(false)
        .alongWith(raiseElevatorAtDistance(l1()))
        .andThen(outtakeL1())

fun alignScoreL2(): Command =
    alignPrep(l2())
        .andThen(alignCommand(false))
        .andThen(outtakeCoralAndDriveBack(false))

fun alignScoreL3(): Command =
    alignPrep(l3())
        .andThen(alignCommand())
        .andThen(outtakeCoralAndDriveBack(false))

fun alignScoreL4(): Command =
    alignPrep(alignmentSetpointL4())
        .andThen(alignCommand())
        .andThen(outtakeCoralAndDriveBack(true))

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
