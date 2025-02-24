package frc.robot.autonomous

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.gripper
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.feeder
import frc.robot.subsystems.l4
import frc.robot.swerveDrive

fun B1L(): Command =
    Commands.sequence(
        Commands.runOnce({
            selectedScorePose = { Reef1 }
            isLeft = { true }
        }),
        alignCommand(::l4).withTimeout(4.0),
        l4(Trigger { true })
    )

fun B1R(): Command =
    Commands.sequence(
        Commands.runOnce({
            selectedScorePose = { Reef1 }
            isLeft = { false }
        }),
        alignCommand(::l4).withTimeout(4.0),
        l4(Trigger { true })
    )

fun C6R(): Command =
    Commands.sequence(
        Commands.runOnce({
            selectedScorePose = { Reef6 }
            isLeft = { false }
        }),
        alignCommand(::l4).withTimeout(4.0),
        l4(Trigger { true })
    )

fun S6L(): Command =
    Commands.sequence(
        Commands.runOnce({
            selectedScorePose = { Reef6 }
            isLeft = { true }
        }),
        alignCommand(::l4).withTimeout(4.0),
        l4(Trigger { true })
    )

fun autoFeed(): Command =
    Commands.sequence(pathFindToPose(FeederRight), feeder(Trigger { true }))

fun C6RL(): Command = Commands.sequence(C6R(), autoFeed(), S6L())

fun dumbAuto(): Command =
    Commands.sequence(
        feeder(Trigger { true }),
        gripper.intake().withTimeout(0.2),
        Commands.runOnce({ swerveDrive.setAngle(Rotation2d.kZero) }),
        WaitCommand(0.8),
        DriveCommands.timedLeave(swerveDrive, 1.2),
        alignToPose(swerveDrive, { true }, ::l4).withTimeout(3.5),
        l4(Trigger { true })
    )
