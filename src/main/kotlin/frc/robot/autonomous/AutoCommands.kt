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
import frc.robot.lib.rotationToPoint
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.swerveDrive
import frc.robot.vision
import java.util.function.Supplier

fun pathFindToPose(pose: Pose2d): Command =
    AutoBuilder.pathfindToPoseFlipped(pose, PATH_CONSTRAINTS, 0.0)

private fun pidDrive(
    drive: Drive,
    controller: PIDController,
    error: () -> Double,
): Command {
    return drive
        .run {
            drive.runVelocity(
                ChassisSpeeds(0.0, controller.calculate(error.invoke()), 0.0)
            )
        }
        .until(controller::atSetpoint)
}

fun align2d(drive: Drive, isLeft: Boolean): Command {
    val rotationController =
        ALIGNMENT_ROTATION_GAINS.run { PIDController(kP, kI, kD) }
    rotationController.setpoint = 0.0
    rotationController.setTolerance(
        ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians)
    )
    val rotationError = { drive.rotation.radians }

    val xController = ALIGNMENT_X_GAINS.run { PIDController(kP, kI, kD) }
    xController.setpoint = if (isLeft) ALIGNED_TX_LEFT else ALIGNED_TX_RIGHT
    xController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    val xError = { vision.getTxToTarget(1).radians }

    return pidDrive(drive, rotationController, rotationError)
        .andThen(
            Commands.run({
                    drive.runVelocity(
                        ChassisSpeeds(
                            ALIGN_SPEED.`in`(Units.MetersPerSecond),
                            0.0,
                            0.0
                        )
                    )
                })
                .until { vision.getTyToTarget(1).radians == ALIGNED_TY }
        )
        .andThen(pidDrive(drive, xController, xError))
}

fun alignToPose(
    drive: Drive,
    robotPoseSupplier: Supplier<Pose2d>,
    targetPoseSupplier: Supplier<Pose2d>
): Command {

    val radialController =
        PIDController(
            ALIGNMENT_X_GAINS.kP,
            ALIGNMENT_X_GAINS.kI,
            ALIGNMENT_X_GAINS.kD
        )
    val rotationController =
        PIDController(
            ALIGNMENT_ROTATION_GAINS.kP,
            ALIGNMENT_ROTATION_GAINS.kI,
            ALIGNMENT_ROTATION_GAINS.kD,
        )
    rotationController.enableContinuousInput(-Math.PI, Math.PI)

    radialController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    rotationController.setTolerance(
        ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians)
    )

    return Commands.run(
            {
                val robotPose = robotPoseSupplier.get()
                val targetPose = targetPoseSupplier.get()

                val radius =
                    (robotPose.translation - targetPose.translation).norm
                val output = radialController.calculate(radius, 0.0)
                val fieldRelativeAngle =
                    targetPose.translation.rotationToPoint(
                        robotPose.translation
                    )
                val targetSpeeds =
                    ChassisSpeeds(
                        output * fieldRelativeAngle.cos,
                        output * fieldRelativeAngle.sin,
                        rotationController.calculate(
                            robotPose.rotation.radians,
                            targetPose.rotation.radians
                        )
                    )

                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        targetSpeeds,
                        if (IS_RED && CURRENT_MODE == Mode.REAL)
                            drive.rotation + Rotation2d.k180deg
                        else drive.rotation
                    )
                )
            },
            swerveDrive
        )
        .until { radialController.atSetpoint() }
        .andThen(Commands.runOnce({ drive.runVelocity(ChassisSpeeds()) }))
}
