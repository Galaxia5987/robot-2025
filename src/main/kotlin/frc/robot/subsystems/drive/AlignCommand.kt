package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import frc.robot.lib.controllers.TunableHolonomicDriveController
import frc.robot.swerveDrive
import org.littletonrobotics.junction.Logger

private val translationController =
    PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD)

private val angularController =
    ProfiledPIDController(
        ANGULAR_KP,
        ANGULAR_KI,
        ANGULAR_KD,
        ANGULAR_CONSTRAINTS
    )

val controller =
    TunableHolonomicDriveController(translationController, angularController)
        .apply { setTolerance(TOLERANCE) }

/**
 * Creates a command that aligns the robot to a given goal pose using a
 * holonomic drive controller.
 *
 * A [Command] that aligns the robot to the specified [goalPose].
 *
 * This command will:
 * - Continuously update the robot's chassis speeds using the provided goal pose
 * and desired linear velocity.
 * - Stop when the robot's pose is within the specified tolerance of the goal.
 *
 * This is useful for aligning the robot to a specific point on the field, often
 * before performing an action like shooting or intaking.
 *
 * ### Example Usage:
 * ```kotlin
 * driverController.circle().whileTrue(
 *     alignToPose(
 *         getPose2d(2.0, 2.0, Rotation2d.fromDegrees(90.0))
 *     )
 * )
 * ```
 * This will continuously align the robot to the pose (2.0, 2.0) while the
 * "circle" button is held.
 *
 * @param goalPose The target pose that the robot should align to.
 * @param linearVelocity The desired linear velocity when driving to the pose.
 * Defaults to 0 m/s.
 * @param tolerance The acceptable tolerance around the goal pose to consider
 * alignment complete. Defaults to [TOLERANCE].
 * @param holonomicController The holonomic controller to use for the alignment.
 * Defaults to [controller]
 */
fun alignToPose(
    goalPose: Pose2d,
    linearVelocity: LinearVelocity = MetersPerSecond.zero(),
    tolerance: Pose2d = TOLERANCE,
    poseSupplier: () -> Pose2d = { swerveDrive.pose },
    holonomicController: Pair<TunableHolonomicDriveController, String> =
        Pair(controller, DEFAULT_CONTROLLER_NAME),
): Command =
    swerveDrive.runOnce {
        controller.setTolerance(tolerance)
        Logger.recordOutput(
            "Alignment/Controllers/CurrentRunningController",
            holonomicController.second
        )
    }
        .andThen(
            swerveDrive.run {
                swerveDrive.limitlessRunVelocity(
                    holonomicController.first.calculate(
                        poseSupplier.invoke(),
                        goalPose,
                        linearVelocity.`in`(MetersPerSecond),
                        goalPose.rotation
                    )
                )
            }
                .until(controller::atReference)
        )
        .withName("Drive/AlignToPose")
