package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.swerveDrive
import frc.robot.vision
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier

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

fun alignToPose(
    drive: Drive,
    robotPose: Supplier<Pose2d>,
    targetPoseSupplier: Supplier<Pose2d>
): Command {
    val xController =
        PIDController(
            3.0,
            0.0,
            0.2
        )
    val yController =
        PIDController(
            3.0,
            0.0,
            0.2
        )
    val rotationController =
        PIDController(
            1.0,
            0.0,
            0.0,
        )
    rotationController.enableContinuousInput(-Math.PI, Math.PI)

    return Commands.run({
        Logger.recordOutput("Xsetpoint", xController.setpoint)
        Logger.recordOutput("Ysetpoint", yController.setpoint)
        Logger.recordOutput(
            "Rotationsetpoint",
            rotationController.setpoint
        )

        val targetPose = targetPoseSupplier.get()

        val targetSpeeds = ChassisSpeeds()
        targetSpeeds.vxMetersPerSecond =
            xController.calculate(robotPose.get().x, targetPose.x)
        targetSpeeds.vyMetersPerSecond =
            yController.calculate(robotPose.get().y, targetPose.y)
        targetSpeeds.omegaRadiansPerSecond =
            rotationController.calculate(
                robotPose.get().rotation.radians,
                targetPose.rotation.radians
            )

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                targetSpeeds,
                drive.rotation
            )
        )
    })
}
