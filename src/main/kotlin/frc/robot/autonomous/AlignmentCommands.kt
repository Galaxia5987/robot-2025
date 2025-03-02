package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.defer
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.elevator
import frc.robot.gripper
import frc.robot.lib.distanceFromPoint
import frc.robot.subsystems.outtakeCoral
import frc.robot.swerveDrive
import frc.robot.wrist
import org.littletonrobotics.junction.Logger

var isAligning = Trigger { alignCommand().isScheduled }

val pose: Pose2d
    get() = swerveDrive.pose

fun alignCommand(): Command {
    return swerveDrive.defer {
        runOnce({
            resetProfiledPID(pose, swerveDrive.chassisSpeeds)
            setGoal(selectedScorePose.invoke())
        })
            .andThen(
                swerveDrive.run {
                    swerveDrive.limitlessRunVelocity(getSpeed(pose).invoke())
                }
            )
    }
}

private val atAlignmentSetpoint = Trigger { atGoal.asBoolean && isAligning.asBoolean }.onTrue(outtakeCoral())

private val isWithinDistance = Trigger {
    swerveDrive.pose.distanceFromPoint(
        selectedScorePose.invoke().translation
    ) <= Units.Meters.of(0.2)
}

private val shouldOpenElevator =
    Trigger { isWithinDistance.asBoolean && isAligning.asBoolean }.onTrue(
        defer(
            { selectedHeightCommand },
            setOf(wrist, elevator, gripper)
        )
    )

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
