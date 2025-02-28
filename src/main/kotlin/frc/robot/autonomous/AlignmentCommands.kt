package frc.robot.autonomous

import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.distanceFromPoint
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.outtakeCoral
import frc.robot.swerveDrive
import org.littletonrobotics.junction.AutoLogOutput

@AutoLogOutput(key = "AutoAlignment/IsAligning")
var isAligning: Trigger = Trigger { false }.whileTrue(
    runOnce({ setGoal(selectedScorePose.invoke()) }).andThen(
        DriveCommands.driveCommand(swerveDrive, getSpeed(swerveDrive.pose).invoke())
    )
)

@AutoLogOutput(key = "AutoAlignment/AtAlignmentSetpoint")
private val atAlignmentSetpoint = Trigger { atGoal() }.and(isAligning).onTrue(outtakeCoral())

private val isWithinDistance = Trigger {
    swerveDrive.pose.distanceFromPoint(
        selectedScorePose.invoke().translation
    ) <= MAX_ALIGNMENT_DISTANCE
}

@AutoLogOutput(key = "AutoAlignment/ShouldOpenElevator")
private val shouldOpenElevator = isWithinDistance.and(isAligning).onTrue(selectedHeightCommand.invoke())