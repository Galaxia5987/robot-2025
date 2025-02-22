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
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.vision
import org.littletonrobotics.junction.Logger

fun pathFindToPose(pose: Pose2d): Command =
    AutoBuilder.pathfindToPoseFlipped(pose, PATH_CONSTRAINTS, 0.0)

fun alignToPose(drive: Drive, isLeft: Boolean, scoreCommand: Command): Command {
    val rotationController =
        ALIGNMENT_ROTATION_GAINS.run { PIDController(kP, kI, kD) }
    rotationController.setpoint = Rotation2d.fromDegrees(155.0).radians
    rotationController.setTolerance(
        ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians)
    )

    val yController = ALIGNMENT_X_GAINS.run { PIDController(kP, kI, kD) }
    yController.setpoint = if (isLeft) -ALIGNED_TX_LEFT else -ALIGNED_TX_RIGHT
    yController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    val yError = { -vision.getTranslationToBestTarget(1).y }

    val xController = ALIGNMENT_Y_GAINS.run { PIDController(kP, kI, kD) }
    xController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    xController.setpoint = 0.0
    val xError = { -vision.getTranslationToBestTarget(1).x }
    return Commands.sequence(
            drive
                .run {
                    drive.runVelocity(
                        ChassisSpeeds(
                            0.0,
                            yController.calculate(yError.invoke()),
                            -rotationController.calculate(
                                vision.getYawToTarget(1).get().radians
                            )
                        )
                    )
                }
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
            drive.run {
                drive.runVelocity(
                    ChassisSpeeds(
                        ALIGNMENT_Y_VELOCITY.`in`(Units.MetersPerSecond),
                        0.0,
                        0.0
                    )
                )
            }
        )
        .alongWith(Commands.run({ Logger.recordOutput("XError", yError) }))
}
