package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.distanceFromPoint
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l3
import frc.robot.subsystems.l4
import frc.robot.subsystems.outtakeCoral
import frc.robot.subsystems.outtakeCoralAndDriveBack
import frc.robot.subsystems.raiseElevatorAtDistance
import frc.robot.swerveDrive
import org.littletonrobotics.junction.Logger

var isAligning = Trigger { alignCommand().isScheduled }

private fun alignToPose(targetPose: Pose2d, endTrigger: Trigger): Command {
    return swerveDrive
        .runOnce{
                isAligning = Trigger { true }
                resetProfiledPID(swerveDrive.pose, swerveDrive.chassisSpeeds)
                setGoal(targetPose)
            }
                .andThen(
                    swerveDrive.run {
                        swerveDrive.limitlessRunVelocity(
                            getSpeed(swerveDrive.pose).invoke()
                        )
                    }
                )
                .until(endTrigger)
        .finallyDo(Runnable { isAligning = Trigger { false } })
}

fun alignCommand(): Command =
    swerveDrive.defer{ alignToPose(selectedScorePose.invoke(), atGoal) }


private fun alignL4Prep(): Command =
    swerveDrive.defer{ alignToPose(selectedScorePose.invoke(), Trigger{false}) }
        .raceWith(raiseElevatorAtDistance(l4()))

fun autoScoreL1(): Command =
    alignCommand()
        .alongWith(raiseElevatorAtDistance(l1()))
        .andThen(outtakeCoral())

fun autoScoreL2(): Command =
    alignCommand()
        .alongWith(raiseElevatorAtDistance(l2()))
        .andThen(outtakeCoral())

fun autoScoreL3(): Command =
    alignCommand()
        .alongWith(raiseElevatorAtDistance(l3()))
        .andThen(outtakeCoral())

fun autoScoreL4(): Command =
    alignL4Prep()
        .andThen(alignCommand())
        .andThen(outtakeCoralAndDriveBack())

private val atAlignmentSetpoint = Trigger {
    atGoal.asBoolean && isAligning.asBoolean
}

private val isWithinDistance = Trigger {
    swerveDrive.pose.distanceFromPoint(
        selectedScorePose.invoke().translation
    ) <= Units.Meters.of(0.4)
}

val shouldOpenElevator = Trigger {
    isWithinDistance.asBoolean && isAligning.asBoolean
}

fun logTriggers() {
    mapOf(
            "IsAligning" to isAligning,
            "AtAlignmentSetpoint" to atAlignmentSetpoint,
            "IsWithinDistance" to isWithinDistance,
            "ShouldOpenElevator" to shouldOpenElevator,
        )
        .forEach { (key, value) ->
            Logger.recordOutput("AutoAlignment/$key", value)
        }
}
