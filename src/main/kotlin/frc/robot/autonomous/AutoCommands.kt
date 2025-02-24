package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.elevator
import frc.robot.gripper
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.swerveDrive
import frc.robot.vision
import frc.robot.wrist
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

fun pathFindToPose(pose: Pose2d): Command =
    AutoBuilder.pathfindToPoseFlipped(pose, PATH_CONSTRAINTS, 0.0)

fun alignToPose(
    drive: Drive,
    isLeft: () -> Boolean,
    scoreCommand: () -> Command
): Command {
    val rotationController =
        ALIGNMENT_ROTATION_GAINS.run { PIDController(kP, kI, kD) }
    rotationController.setpoint = ALIGNED_ROTATION.radians
    rotationController.setTolerance(
        ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians)
    )
    //    val yawToTarget = {
    //        vision.getYawToTarget(VisionConstants.frontCameraIndex).get().radians
    //    }

    val yController = ALIGNMENT_Y_GAINS.run { PIDController(kP, kI, kD) }
    yController.setpoint =
        if (isLeft.invoke()) -ALIGNED_Y_LEFT else -ALIGNED_Y_RIGHT
    yController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    val yError = {
        -vision.getTranslationToBestTarget(VisionConstants.frontCameraIndex).y
    }

    return Commands.sequence(
            Commands.runOnce({ aligning = true }),
            drive
                .run {
                    drive.runVelocity(
                        ChassisSpeeds(
                            0.0,
                            yController.calculate(yError.invoke()),
                            -rotationController.calculate(
                                vision
                                    .getYawToTarget(
                                        VisionConstants.frontCameraIndex
                                    )
                                    .get()
                                    .radians
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
                .alongWith(scoreCommand.invoke()),
            drive.run {
                drive.runVelocity(
                    ChassisSpeeds(
                        ALIGNMENT_FORWARD_VELOCITY.`in`(Units.MetersPerSecond),
                        0.0,
                        0.0
                    )
                )
            }
            //            .withTimeout(10.0)
            )
        .finallyDo(Runnable { aligning = false })
        .alongWith(
            Commands.run({
                Logger.recordOutput("Auto Alignment/YError", yError)
            })
        )
}

fun alignCommand(scoreCommand: () -> Command): Command =
    Commands.defer(
        {
            pathFindToPose(selectedScorePose.invoke())
                .andThen(alignToPose(swerveDrive, isLeft, scoreCommand))
        },
        setOf(swerveDrive, elevator, gripper, wrist)
    )

private var aligning = false

val IS_ALIGNING = Trigger { aligning }

@AutoLogOutput(key = "Auto Alignment/is aligning")
private fun getIsAligning() = IS_ALIGNING
