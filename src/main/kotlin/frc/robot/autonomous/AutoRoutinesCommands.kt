package frc.robot.autonomous

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.dumbL4
import frc.robot.subsystems.feeder
import frc.robot.subsystems.l4
import frc.robot.swerveDrive

fun B1L(): Command =
    Commands.sequence(
        Commands.runOnce({
            selectedScorePose = { Reef1 }
            isLeft = { true }
        }),
        alignCommandWithPath(::l4),
        l4(Trigger { true })
    )

fun B1R(): Command =
    Commands.sequence(
        Commands.runOnce({
            selectedScorePose = { Reef1 }
            isLeft = { false }
        }),
        alignCommandWithPath(::l4),
        l4(Trigger { true })
    )

fun C6L(): Command =
    Commands.sequence(
        Commands.runOnce({
            selectedScorePose = { Reef6 }
            isLeft = { true }
        }),
        alignCommandWithPath(::l4),
        dumbL4(Trigger { true })
    )

fun S5R(): Command =
    Commands.sequence(
        Commands.runOnce({
            selectedScorePose = { Reef5 }
            isLeft = { false }
        }),
        alignCommandWithPath(::l4),
        l4(Trigger { true })
    )

fun autoFeed(): Command =
    Commands.sequence(
        Commands.run({ swerveDrive.runVelocity(ChassisSpeeds(-1.0, 0.0, 2.8)) })
            .withTimeout(0.5),
        pathFindToPose(FeederRight),
        DriveCommands.joystickDrive(swerveDrive, { 0.0 }, { 0.0 }, { 0.0 })
            .withTimeout(0.02),
        Commands.run({ swerveDrive.runVelocity(ChassisSpeeds(1.2, 0.0, 0.0)) })
            .raceWith(feeder(Trigger { true }))
    )

fun C6L5R(): Command = Commands.sequence(C6L(), autoFeed(), S5R())

fun dumbAuto(): Command =
    Commands.sequence(
        Commands.runOnce({ swerveDrive.setAngle(Rotation2d.kZero) }),
        WaitCommand(3.0),
        DriveCommands.timedLeave(swerveDrive, 1.2),
        alignToPose(swerveDrive, { true }, ::l4),
        dumbL4(Trigger { true })
    )
