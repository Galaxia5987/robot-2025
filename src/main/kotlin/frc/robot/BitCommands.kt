package frc.robot

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l2algaePickup
import frc.robot.subsystems.l3
import frc.robot.subsystems.l4
import frc.robot.subsystems.moveDefaultPosition

fun intakeBit(): Command =
    sequence(
            intakeAlgae().withTimeout(2.4),
            roller.outtake().withTimeout(Seconds.of(0.8)),
            extender.retract().withTimeout(0.5)
        )
        .withName("Bits/Intake")

fun driveBit(): Command =
    sequence(
        DriveCommands.driveCommand(swerveDrive, ChassisSpeeds(2.0, 0.0, 0.0))
            .withTimeout(2.5),
        DriveCommands.driveCommand(swerveDrive, ChassisSpeeds(0.0, 2.0, 0.0))
            .withTimeout(2.5),
        DriveCommands.driveCommand(swerveDrive, ChassisSpeeds(-2.0, 0.0, 0.0))
            .withTimeout(2.5),
        DriveCommands.driveCommand(swerveDrive, ChassisSpeeds(0.0, -2.0, 0.0))
            .withTimeout(2.5),
        DriveCommands.driveCommand(swerveDrive, ChassisSpeeds(0.0, 0.0, 0.0))
            .withTimeout(0.1)
    )

fun elevatorWristBit(): Command =
    sequence(
        l1(),
        WaitCommand(1.0),
        l2(),
        WaitCommand(1.0),
        l3(),
        WaitCommand(1.0),
        l4(),
        WaitCommand(1.0),
        moveDefaultPosition(true),
        WaitCommand(1.0),
        gripper.intake().withTimeout(3.0),
        l4(),
        WaitCommand(1.0),
        moveDefaultPosition(true)
    )

fun algaeBit(): Command =
    sequence(
        l2algaePickup(),
        WaitCommand(2.5),
        gripper.outtake().withTimeout(0.2)
    )

fun climbBit(): Command =
    sequence(
        climber.powerControl { 1.0 }.withTimeout(1.5),
        climber.powerControl { -1.0 }.withTimeout(1.5)
    )

fun runAllBits(): Command =
    sequence(
        driveBit(),
        intakeBit(),
        elevatorWristBit(),
        algaeBit(),
        climbBit()
    )
