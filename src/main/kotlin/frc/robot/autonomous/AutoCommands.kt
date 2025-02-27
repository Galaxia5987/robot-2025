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
import frc.robot.IS_RED
import frc.robot.elevator
import frc.robot.gripper
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.subsystems.moveDefaultPosition
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.swerveDrive
import frc.robot.vision
import frc.robot.wrist
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

private var lastYError = 0.0
private var lastXError = 0.0
private var lastRotationError = Rotation2d()

private val DESIRED_TAG // TODO: Change 22 back to 21
    get() = if (IS_RED) 10 else 22

fun pathFindToPose(pose: Pose2d): Command =
    AutoBuilder.pathfindToPoseFlipped(pose, PATH_CONSTRAINTS, 0.0)

fun alignToPose(
    drive: Drive,
    isLeft: () -> Boolean,
    scoreCommand: () -> Command,
): Command {
    val rotationController =
        ALIGNMENT_ROTATION_GAINS.run { PIDController(kP, kI, kD) }
    rotationController.setpoint = ALIGNED_ROTATION.radians
    rotationController.setTolerance(
        ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians)
    )

    val yController = ALIGNMENT_Y_GAINS.run { PIDController(kP, kI, kD) }
    yController.setpoint =
        if (isLeft.invoke()) -ALIGNED_Y_LEFT else -ALIGNED_Y_RIGHT
    yController.setTolerance(Y_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    val yError = {
        val newTranslation =
            vision.getTransformToID(
                VisionConstants.frontCameraIndex,
                DESIRED_TAG
            )
        if (newTranslation != null) {
            lastYError = -newTranslation.y
        }
        lastYError
    }

    val rotationError = {
        val newTransform =
            vision.getTransformToID(
                VisionConstants.frontCameraIndex,
                DESIRED_TAG
            )
        if (newTransform != null) {
            lastRotationError = newTransform.rotation.toRotation2d()
        }
        lastRotationError
    }

    val xController = ALIGNMENT_X_GAINS.run { PIDController(kP, kI, kD) }
    xController.setpoint = X_ALIGNMENT_OFFSET.`in`(Units.Meters)
    xController.setTolerance(X_ALIGNMENT_TOLERANCE.`in`(Units.Meters))
    val xError = {
        val newTranslation =
            vision.getTransformToID(
                VisionConstants.frontCameraIndex,
                DESIRED_TAG
            )
        if (newTranslation != null) {
            lastXError = -newTranslation.x
        }
        lastXError
    }

    return Commands.sequence(
            Commands.runOnce({
                aligning = true
                shouldScore = { false }
            }),
            drive
                .run {
                    drive.runVelocity(
                        if (
                            vision.getBestTargetID(
                                VisionConstants.frontCameraIndex
                            ) == DESIRED_TAG
                        )
                            ChassisSpeeds(
                                xController.calculate(xError.invoke()),
                                yController.calculate(yError.invoke()),
                                -rotationController.calculate(
                                    rotationError.invoke().radians
                                )
                            )
                        else {
                            ChassisSpeeds(0.0, 0.0, 0.0)
                        }
                    )
                }
                .until(
                    Trigger {
                            yController.atSetpoint() &&
                                rotationController.atSetpoint() &&
                                xController.atSetpoint()
                        }
                        .debounce(0.5)
                ),
            Commands.runOnce({ shouldScore = { true } }),
            drive
                .runOnce {
                    rotationController.setpoint = drive.rotation.radians
                    drive.setAngle(Rotation2d.kZero)
                }
                .alongWith(scoreCommand.invoke()),
            WaitCommand(0.4),
            drive
                .run {
                    drive.runVelocity(
                        ChassisSpeeds(
                            ALIGNMENT_FORWARD_VELOCITY.`in`(
                                Units.MetersPerSecond
                            ),
                            0.0,
                            0.0
                        )
                    )
                }
                .withTimeout(1.0)
        )
        .finallyDo(Runnable { aligning = false })
        .raceWith(
            Commands.run({
                Logger.recordOutput("Auto Alignment/lastXError", lastXError)
                Logger.recordOutput("Auto Alignment/lastYError", lastYError)
                Logger.recordOutput("Auto Alignment/lastRotationError", lastRotationError)
            })
        )
}

fun finishAlign(scoreCommand: (Trigger) -> Command): Command =
    Commands.defer(
        {
            Commands.either(
                scoreCommand(Trigger { true }),
                moveDefaultPosition(),
                shouldScore
            )
        },
        setOf(elevator, wrist, gripper)
    )

fun alignCommand(scoreCommand: () -> Command): Command =
    Commands.defer(
        { alignToPose(swerveDrive, isLeft, scoreCommand) },
        setOf(swerveDrive, elevator, gripper, wrist)
    )

fun alignCommandWithPath(scoreCommand: () -> Command): Command =
    Commands.defer(
        {
            pathFindToPose(selectedScorePose.invoke())
                .andThen(alignToPose(swerveDrive, isLeft, scoreCommand))
        },
        setOf(swerveDrive, elevator, gripper, wrist)
    )

private var aligning = false

var shouldScore = { false }
val IS_ALIGNING = Trigger { aligning }

@AutoLogOutput(key = "Auto Alignment/is aligning")
private fun getIsAligning() = IS_ALIGNING
