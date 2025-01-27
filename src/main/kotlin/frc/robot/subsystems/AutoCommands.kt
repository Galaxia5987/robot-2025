package frc.robot.subsystems

import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_RED
import frc.robot.lib.distanceFromPoint
import frc.robot.swerveDrive

private val autoFactory =
    AutoFactory(
        swerveDrive::getPose,
        swerveDrive::setPose,
        swerveDrive::followPath,
        IS_RED,
        swerveDrive
    )

private fun AutoTrajectory.atDistanceFromFinalPoint(
    robotPose: () -> Pose2d,
    distance: Distance,
    tolerance: Distance
): Trigger = Trigger {
    MathUtil.isNear(
        robotPose.invoke().distanceFromPoint(this.finalPose.get().translation).`in`(Units.Meters),
        distance.`in`(Units.Meters),
        tolerance.`in`(Units.Meters)
    )
}

fun ALeave(): AutoRoutine {
    val routine = autoFactory.newRoutine("A Leave")
    val trajectory = routine.trajectory("A Leave")

    routine
        .active()
        .onTrue(trajectory.resetOdometry().alongWith(trajectory.cmd()))

    return routine
}

fun BLeave(): AutoRoutine {
    val routine = autoFactory.newRoutine("B Leave")
    val trajectory = routine.trajectory("B Leave")

    routine
        .active()
        .onTrue(trajectory.resetOdometry().alongWith(trajectory.cmd()))

    return routine
}

fun CLeave(): AutoRoutine {
    val routine = autoFactory.newRoutine("C Leave")
    val trajectory = routine.trajectory("C Leave")

    routine
        .active()
        .onTrue(trajectory.resetOdometry().alongWith(trajectory.cmd()))

    return routine
}

fun A2R3LR(): AutoRoutine {
    val routine = autoFactory.newRoutine("A2R3LR")

    val A2R = routine.trajectory("A2R")
    val t2RS = routine.trajectory("2RS")
    val S3L = routine.trajectory("S3L")
    val t3LS = routine.trajectory("3LS")
    val S3R = routine.trajectory("S3R")

    routine.active().onTrue(A2R.cmd())
    A2R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_2R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(A2R.done()))

    A2R.done().onTrue(t2RS.cmd())
    t2RS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t2RS.done()))

    t2RS.done().onTrue(S3L.cmd())
    S3L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_3L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S3L.done()))

    S3L.done().onTrue(t3LS.cmd())
    t3LS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t3LS.done()))

    t3LS.done().onTrue(S3R.cmd())
    S3R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_3R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S3R.done()))

    return routine
}


fun A3LR4L(): AutoRoutine {
    val routine = autoFactory.newRoutine("A3LR4L")

    val A3L = routine.trajectory("A3L")
    val t3LS = routine.trajectory("3LS")
    val S3R = routine.trajectory("S3R")
    val t3RS = routine.trajectory("3RS")
    val S4L = routine.trajectory("S4L")

    routine.active().onTrue(A3L.cmd())
    A3L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_3L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(A3L.done()))

    A3L.done().onTrue(t3LS.cmd())
    t3LS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t3LS.done()))

    t3LS.done().onTrue(S3R.cmd())
    S3R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_3R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S3R.done()))

    S3R.done().onTrue(t3RS.cmd())
    t3RS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t3RS.done()))

    t3RS.done().onTrue(S4L.cmd())
    S4L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_4L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S4L.done()))

    return routine
}

fun B1L6RL(): AutoRoutine {
    val routine = autoFactory.newRoutine("B1L6RL")

    val B1L = routine.trajectory("B1L")
    val t1LS = routine.trajectory("1LS")
    val S6R = routine.trajectory("S6R")
    val t6RS = routine.trajectory("6RS")
    val S6L = routine.trajectory("S6L")

    routine.active().onTrue(B1L.cmd())
    B1L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_1L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(B1L.done()))

    B1L.done().onTrue(t1LS.cmd())
    t1LS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t1LS.done()))

    t1LS.done().onTrue(S6R.cmd())
    S6R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_6R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S6R.done()))

    S6R.done().onTrue(t6RS.cmd())
    t6RS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t6RS.done()))

    t6RS.done().onTrue(S6L.cmd())
    S6L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_6L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S6L.done()))

    return routine
}

fun B1R2LR(): AutoRoutine {
    val routine = autoFactory.newRoutine("B1R2LR")

    val B1R = routine.trajectory("B1R")
    val t1RS = routine.trajectory("1RS")
    val S2L = routine.trajectory("S2L")
    val t3LS = routine.trajectory("3LS")
    val S2R = routine.trajectory("S2R")

    routine.active().onTrue(B1R.cmd())
    B1R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_1R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(B1R.done()))

    B1R.done().onTrue(t1RS.cmd())
    t1RS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t1RS.done()))

    t1RS.done().onTrue(S2L.cmd())
    S2L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_2L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S2L.done()))

    S2L.done().onTrue(t3LS.cmd())
    t3LS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t3LS.done()))

    t3LS.done().onTrue(S2R.cmd())
    S2R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_2R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S2R.done()))

    return routine
}

fun B1R(): AutoRoutine {
    val routine = autoFactory.newRoutine("B1R")

    val B1R = routine.trajectory("B1R")

    routine.active().onTrue(B1R.cmd())
    B1R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_1R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(B1R.done()))

    return routine
}

fun C5RL4R(): AutoRoutine {
    val routine = autoFactory.newRoutine("C5RL4R")

    val C5R = routine.trajectory("C5R")
    val t5RS = routine.trajectory("5RS")
    val S5L = routine.trajectory("S5L")
    val t5LS = routine.trajectory("5LS")
    val S4L = routine.trajectory("S4L")

    routine.active().onTrue(C5R.cmd())
    C5R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_5R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(C5R.done()))

    C5R.done().onTrue(t5RS.cmd())
    t5RS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t5RS.done()))

    t5RS.done().onTrue(S5L.cmd())
    S5L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_5L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S5L.done()))

    S5L.done().onTrue(t5LS.cmd())
    t5LS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t5LS.done()))

    t5LS.done().onTrue(S4L.cmd())
    S4L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_4L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S4L.done()))

    return routine
}

fun C6L5RL(): AutoRoutine {
    val routine = autoFactory.newRoutine("C6L5RL")

    val C6L = routine.trajectory("C6L")
    val t6LS = routine.trajectory("6LS")
    val S5R = routine.trajectory("S5R")
    val t5RS = routine.trajectory("5RS")
    val S5L = routine.trajectory("S5L")

    routine.active().onTrue(C6L.cmd())
    C6L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_6L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(C6L.done()))

    C6L.done().onTrue(t6LS.cmd())
    t6LS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t6LS.done()))

    t6LS.done().onTrue(S5R.cmd())
    S5R.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_5R_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(l4(S5R.done()))

    S5R.done().onTrue(t5RS.cmd())
    t5RS.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_FEEDER_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(t5RS.done()))

    t5RS.done().onTrue(S5L.cmd())
    S5L.atDistanceFromFinalPoint(
        swerveDrive::getPose,
        SCORE_5L_DISTANCE,
        AUTO_DISTANCE_TOLERANCE
    ).onTrue(feeder(S5L.done()))

    return routine
}

val autoRoutines =
    arrayOf(A2R3LR(), A3LR4L(), ALeave(), BLeave(), CLeave(), B1L6RL(), B1R2LR(), B1R(), C5RL4R(), C6L5RL())