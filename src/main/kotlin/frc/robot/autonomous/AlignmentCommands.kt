package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.defer
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.distanceFromPoint
import frc.robot.subsystems.outtakeCoral
import frc.robot.swerveDrive
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

@AutoLogOutput(key = "AutoAlignment/IsAligning")
var isAligning = Trigger { alignCommand().isScheduled }

val pose: Pose2d
    get() = swerveDrive.pose

fun alignCommand(): Command {
    return defer(
        {
            runOnce({
                    resetProfiledPID(pose, swerveDrive.chassisSpeeds)
                    setGoal(selectedScorePose.invoke())
                })
                .andThen(
                    swerveDrive.run {
                        swerveDrive.normalRunVelocity(getSpeed(pose).invoke())
                    }
                )
        },
        setOf(swerveDrive)
    )
}

@AutoLogOutput(key = "AutoAlignment/AtAlignmentSetpoint")
private val atAlignmentSetpoint =
    Trigger { atGoal.asBoolean && isAligning.asBoolean }.onTrue(outtakeCoral())

private val isWithinDistance = Trigger {
    swerveDrive.pose.distanceFromPoint(
        selectedScorePose.invoke().translation
    ) <= Units.Meters.of(0.2)
}

@AutoLogOutput(key = "AutoAlignment/ShouldOpenElevator")
private val shouldOpenElevator =
    Trigger { isWithinDistance.asBoolean && isAligning.asBoolean }
        .onTrue(selectedHeightCommand.invoke())

fun logTriggers() {
    Logger.recordOutput("AutoAlignment/IsAligning", isAligning)
    Logger.recordOutput(
        "AutoAlignment/AtAlignmentSetpoint",
        atAlignmentSetpoint
    )
    Logger.recordOutput("AutoAlignment/IsWithinDistance", isWithinDistance)
    Logger.recordOutput("AutoAlignment/ShouldOpenElevator", shouldOpenElevator)
}
