package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.CURRENT_MODE
import frc.robot.IS_RED
import frc.robot.Mode
import frc.robot.lib.distanceFromPoint
import frc.robot.lib.getPose2d
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.swerveDrive
import frc.robot.vision
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import org.littletonrobotics.junction.Logger

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
    if (
        robotPose
            .get()
            .distanceFromPoint(targetPoseSupplier.get().translation) >
            MAX_ALIGNMENT_DISTANCE
    )
        return Commands.none()

    val xController = PIDController(5.0, 0.0, 0.2)
    val yController = PIDController(5.0, 0.0, 0.2)
    val rotationController =
        PIDController(
            5.0,
            0.0,
            0.2,
        )
    rotationController.enableContinuousInput(-Math.PI, Math.PI)

    return Commands.run({
        Logger.recordOutput(
            "PIDsetpoint",
            getPose2d(
                xController.setpoint,
                yController.setpoint,
                Rotation2d.fromRadians(rotationController.setpoint)
            )
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
                if (IS_RED && CURRENT_MODE == Mode.REAL)
                    drive.rotation.plus(Rotation2d(Math.PI))
                else drive.rotation
            )
        )
    }, swerveDrive)
}
