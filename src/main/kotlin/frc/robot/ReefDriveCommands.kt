package frc.robot

import edu.wpi.first.networktables.NetworkTablesJNI
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

private val REEF_MAP =
    mapOf(
        0 to Commands.none(), // TODO: Fill in actual command
        1 to Commands.none(), // TODO: Fill in actual command
        2 to Commands.none(), // TODO: Fill in actual command
        3 to Commands.none(), // TODO: Fill in actual command
        4 to Commands.none(), // TODO: Fill in actual command
        5 to Commands.none(), // TODO: Fill in actual command
        6 to Commands.none(), // TODO: Fill in actual command
        7 to Commands.none(), // TODO: Fill in actual command
        8 to Commands.none(), // TODO: Fill in actual command
        9 to Commands.none(), // TODO: Fill in actual command
        10 to Commands.none(), // TODO: Fill in actual command
        11 to Commands.none(), // TODO: Fill in actual command
        12 to Commands.none(), // TODO: Fill in actual command (Coral Station)
        13 to Commands.none() // TODO: Fill in actual command (Coral Station)
    )

fun getReefDriveCommand(): Command = REEF_MAP[NetworkTablesJNI.getEntry(NetworkTablesJNI.getDefaultInstance(), "Dashboard/TargetReefPose")]!!