package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import frc.robot.lib.debounce
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.TunerConstants.PATH_CONSTRAINTS
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.swerveDrive
import frc.robot.vision
import org.littletonrobotics.junction.AutoLogOutput

fun pathFindToPose(pose: Pose2d): Command = AutoBuilder.pathfindToPoseFlipped(pose, PATH_CONSTRAINTS, 0.0)

@AutoLogOutput(key = "Auto Alignment/Is aligning")
private var aligning = false

@AutoLogOutput(key = "Auto alignment/Yaw error")
private val yawError = { -vision.getYawToTarget(VisionConstants.frontCameraIndex).radians }

@AutoLogOutput(key = "Auto alignment/Y error")
private val yError = { -vision.getTranslationToBestTarget(VisionConstants.frontCameraIndex).y }

fun alignToPose(
    isLeft: Boolean,
    scoreCommand: Command,
): Command {
    val moveSlowlyForward = DriveCommands.runVelocity(
        ChassisSpeeds(
            ALIGNMENT_FORWARD_VELOCITY.`in`(Units.MetersPerSecond), 0.0, 0.0
        )
    )
    return sequence(
        runOnce({ aligning = true }),
        pidDrive(isLeft),
        DriveCommands.setAngle(Rotation2d.kZero).alongWith(scoreCommand),
        moveSlowlyForward.withTimeout(10.0)
    ).finallyDo(Runnable { aligning = false })
}

private fun pidDrive(isLeft: Boolean): ParallelRaceGroup? {
    val rotationController = ALIGNMENT_ROTATION_GAINS.run { PIDController(kP, kI, kD) }
    rotationController.setpoint = ALIGNED_ROTATION.radians
    rotationController.setTolerance(
        ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians)
    )

    val yController = ALIGNMENT_Y_GAINS.run { PIDController(kP, kI, kD) }
    yController.setpoint = if (isLeft) -ALIGNED_Y_LEFT else -ALIGNED_Y_RIGHT
    yController.setTolerance(LINEAR_ALIGNMENT_TOLERANCE.`in`(Units.Meters))

    val runPid = DriveCommands.runVelocity(
        ChassisSpeeds(
            0.0, yController.calculate(yError()), rotationController.calculate(
                yawError()
            )
        )
    ).repeatedly().until({ yController.atSetpoint() && rotationController.atSetpoint() }.debounce(0.15))
    return runPid
}

fun alignCommand(scoreCommand: Command): Command = swerveDrive.defer {
    pathFindToPose(selectedScorePose)
}.andThen(alignToPose(isLeft, scoreCommand))
