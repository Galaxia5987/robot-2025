package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

private val REEF_MAP =
    mapOf(
        0 to REEF_3L_POSE, // TODO: Fill in actual command
        1 to REEF_4R_POSE, // TODO: Fill in actual command
        2 to REEF_4L_POSE, // TODO: Fill in actual command
        3 to REEF_5R_POSE, // TODO: Fill in actual command
        4 to REEF_5L_POSE, // TODO: Fill in actual command
        5 to REEF_6R_POSE, // TODO: Fill in actual command
        6 to REEF_6L_POSE, // TODO: Fill in actual command
        7 to REEF_1R_POSE, // TODO: Fill in actual command
        8 to REEF_1L_POSE, // TODO: Fill in actual command
        9 to REEF_2R_POSE, // TODO: Fill in actual command
        10 to REEF_2L_POSE, // TODO: Fill in actual command
        11 to REEF_3R_POSE, // TODO: Fill in actual command
        12 to RIGHT_FEEDER_POSE, // TODO: Fill in actual command (Coral Station)
        13 to LEFT_FEEDER_POSE // TODO: Fill in actual command (Coral Station)
    )

private fun getDriveToPose(): Pose3d =
    REEF_MAP[networkTables.getIntFromTopic("/Dashboard/TargetReefPose")]!!

fun getDriveToCommand(): Command = Commands.none() // TODO: Implement
