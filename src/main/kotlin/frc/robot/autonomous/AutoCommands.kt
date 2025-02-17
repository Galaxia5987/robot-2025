package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.CURRENT_MODE
import frc.robot.IS_RED
import frc.robot.Mode
import frc.robot.lib.distanceFromPoint
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.swerveDrive
import java.util.function.Supplier
import org.littletonrobotics.junction.Logger

fun pathFindToPose(pose: Pose2d): Command =
    AutoBuilder.pathfindToPoseFlipped(pose, PATH_CONSTRAINTS, 0.0)

fun alignToPose(
    drive: Drive,
    robotPoseSupplier: Supplier<Pose2d>,
    targetPoseSupplier: Supplier<Pose2d>
): Command {

    val xController =
        PIDController(
            ALIGNMENT_X_GAINS.kP,
            ALIGNMENT_X_GAINS.kI,
            ALIGNMENT_X_GAINS.kD
        )
    val yController =
        PIDController(
            ALIGNMENT_Y_GAINS.kP,
            ALIGNMENT_Y_GAINS.kI,
            ALIGNMENT_Y_GAINS.kD
        )
    val rotationController =
        PIDController(
            ALIGNMENT_ROTATION_GAINS.kP,
            ALIGNMENT_ROTATION_GAINS.kI,
            ALIGNMENT_ROTATION_GAINS.kD,
        )
    rotationController.enableContinuousInput(-Math.PI, Math.PI)

    xController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    yController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    rotationController.setTolerance(
        ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians)
    )

    return Commands.run(
        {
            val robotPose = robotPoseSupplier.get()
            val targetPose = targetPoseSupplier.get()

            Logger.recordOutput(
                "Odometry/AlignmentSetpoint",
                Pose2d(
                    xController.setpoint,
                    yController.setpoint,
                    Rotation2d.fromRadians(rotationController.setpoint)
                )
            )

            val targetSpeeds =
                ChassisSpeeds(
                    xController.calculate(robotPose.x, targetPose.x),
                    yController.calculate(robotPose.y, targetPose.y),
                    rotationController.calculate(
                        robotPose.rotation.radians,
                        targetPose.rotation.radians
                    )
                )

            drive.runVelocity(
                if (
                    robotPose.distanceFromPoint(targetPose.translation) <=
                        MAX_ALIGNMENT_DISTANCE
                )
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        targetSpeeds,
                        if (IS_RED && CURRENT_MODE == Mode.REAL)
                            drive.rotation + Rotation2d.k180deg
                        else drive.rotation
                    )
                else ChassisSpeeds()
            )
        },
        swerveDrive
    )
}
