package frc.robot.subsystems

import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_RED
import frc.robot.lib.distanceFromPoint
import frc.robot.swerveDrive

private val autoFactory = AutoFactory(
    swerveDrive::getPose, swerveDrive::setPose, swerveDrive::followPath, IS_RED, swerveDrive
)

private fun AutoTrajectory.atDistanceFromPoint(robotPose: ()-> Pose2d, point: Translation2d, distance: Distance, tolerance: Distance): Trigger =
    Trigger {MathUtil.isNear(robotPose.invoke().distanceFromPoint(point).`in`(Units.Meters), distance.`in`(Units.Meters), tolerance.`in`(Units.Meters))}

fun ALeave(): AutoRoutine {
    val routine = autoFactory.newRoutine("A Leave")
    val trajectory = routine.trajectory("A Leave")

    routine.active().onTrue(
        trajectory.resetOdometry()
            .alongWith(trajectory.cmd())
    )

    return routine
}

fun BLeave(): AutoRoutine {
    val routine = autoFactory.newRoutine("B Leave")
    val trajectory = routine.trajectory("B Leave")

    routine.active().onTrue(
        trajectory.resetOdometry()
            .alongWith(trajectory.cmd())
    )

    return routine
}

fun CLeave(): AutoRoutine {
    val routine = autoFactory.newRoutine("C Leave")
    val trajectory = routine.trajectory("C Leave")

    routine.active().onTrue(
        trajectory.resetOdometry()
            .alongWith(trajectory.cmd())
    )

    return routine
}

fun A2R3LR(): AutoRoutine { //TODO: finish this
    val routine = autoFactory.newRoutine("A2R3LR")

    val A2R = routine.trajectory("A2R")
    val t2RS = routine.trajectory("2RS")
    val S3L = routine.trajectory("S3L")
    val tLS = routine.trajectory("3LS")
    val S3R = routine.trajectory("S3R")

    routine.active().onTrue(
        A2R.cmd()
    )
    A2R.atDistanceFromPoint(
        swerveDrive::getPose,
        SCORE_2R_POINT,
        SCORE_2R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    )

    return routine
}
