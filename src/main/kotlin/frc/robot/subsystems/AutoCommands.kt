package frc.robot.subsystems

import choreo.auto.AutoFactory
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.IS_RED
import frc.robot.swerveDrive

private val autoFactory = AutoFactory(
    swerveDrive::getPose, swerveDrive::setPose, swerveDrive::followPath, IS_RED, swerveDrive
)

fun ALeave(): Command =
    autoFactory.trajectoryCmd("A Leave")

fun BLeave(): Command =
    autoFactory.trajectoryCmd("B Leave")

fun CLeave(): Command =
    autoFactory.trajectoryCmd("C Leave")

fun A2R(): Command =
    autoFactory.trajectoryCmd("A2R")
        .alongWith()