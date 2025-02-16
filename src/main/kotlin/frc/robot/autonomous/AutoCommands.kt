package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.swerveDrive
import frc.robot.vision
import java.util.function.DoubleSupplier

fun alignWithBestVisionTarget(
    cameraIndex: Int,
    ySupplier: DoubleSupplier,
    xSupplier: DoubleSupplier
): Command =
    DriveCommands.joystickDriveAtAngle(swerveDrive, ySupplier, xSupplier) {
        vision.getYawToTarget(cameraIndex)
    }

fun pathFindToPose(pose: Pose2d): Command =
    AutoBuilder.pathfindToPoseFlipped(pose, PATH_CONSTRAINTS, 0.0)
