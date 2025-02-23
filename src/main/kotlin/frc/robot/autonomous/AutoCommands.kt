package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.vision
import org.littletonrobotics.junction.Logger

fun pathFindToPose(pose: Pose2d): Command =
    AutoBuilder.pathfindToPoseFlipped(pose, PATH_CONSTRAINTS, 0.0)

fun alignToPose(drive: Drive, isLeft: Boolean, scoreCommand: Command): Command {
    val rotationController =
        ALIGNMENT_ROTATION_GAINS.run { PIDController(kP, kI, kD) }
    rotationController.setpoint = ALIGNED_ROTATION.radians
    rotationController.setTolerance(
        ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians)
    )
    val yawToTarget = {vision.getYawToTarget(VisionConstants.frontCameraIndex).get().radians}

    val yController = ALIGNMENT_Y_GAINS.run { PIDController(kP, kI, kD) }
    yController.setpoint = if (isLeft) -ALIGNED_Y_LEFT else -ALIGNED_Y_RIGHT
    yController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    val yError = {
        -vision.getTranslationToBestTarget(VisionConstants.frontCameraIndex).y
    }

    return Commands.sequence(
            Commands.runOnce({ aligning = true }),
            DriveCommands.driveCommand(
                    drive,
                    ChassisSpeeds(
                        0.0,
                        yController.calculate(yError.invoke()),
                        -rotationController.calculate(yawToTarget.invoke())
                    )
                )
                .until(
                    Trigger {
                            yController.atSetpoint() &&
                                rotationController.atSetpoint()
                        }
                        .debounce(0.15)
                ),
            drive
                .runOnce { drive.setAngle(Rotation2d.kZero) }
                .alongWith(scoreCommand),
            WaitCommand(0.3),
            DriveCommands.driveCommand(
                drive,
                ChassisSpeeds(
                    ALIGNMENT_FORWARD_VELOCITY.`in`(Units.MetersPerSecond),
                    0.0,
                    0.0
                )
            ).withTimeout(10.0)
        )
        .apply { handleInterrupt { aligning = false } }
        .alongWith(
            Commands.run({
                Logger.recordOutput("Auto Alignment/YError", yError)
            })
        )
}

private var aligning = false

val IS_ALIGNING = Trigger { aligning }
