package frc.robot.subsystems

import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
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
    distance: Distance
): Trigger = Trigger {
    MathUtil.isNear(
        swerveDrive.pose
            .distanceFromPoint(this.finalPose.get().translation)
            .`in`(Units.Meters),
        distance.`in`(Units.Meters),
        AUTO_DISTANCE_TOLERANCE.`in`(Units.Meters)
    )
}

private fun AutoTrajectory.scoreAtDistance(
    distance: Distance = SCORE_DISTANCE,
    outtakeTrigger: Trigger = this.done()
) {
    this.atDistanceFromFinalPoint(distance).onTrue(l4(outtakeTrigger))
}

private fun AutoTrajectory.scoreAtTime(
    time: Time = SCORE_TIME,
    outtakeTrigger: Trigger = this.done()
) {
    this.atTime(time.`in`(Units.Seconds)).onTrue(l4(outtakeTrigger))
}

private fun AutoTrajectory.feedAtDistance(
    distance: Distance = INTAKE_FEEDER_DISTANCE,
    intakeTrigger: Trigger = this.done()
) {
    this.atDistanceFromFinalPoint(distance).onTrue(feeder(intakeTrigger))
}

private fun AutoTrajectory.feedAtTime(
    time: Time = INTAKE_FEEDER_TIME,
    intakeTrigger: Trigger = this.done()
) {
    this.atTime(time.`in`(Units.Seconds)).onTrue(feeder(intakeTrigger))
}

private fun leaveRoutine(startingPoint: String): AutoRoutine {
    val routine = autoFactory.newRoutine("$startingPoint Leave")
    val trajectory = routine.trajectory("$startingPoint Leave")

    routine
        .active()
        .onTrue(trajectory.resetOdometry().andThen(trajectory.cmd()))
    return routine
}

fun ALeave(): AutoRoutine = leaveRoutine("A")

fun BLeave(): AutoRoutine = leaveRoutine("B")

fun CLeave(): AutoRoutine = leaveRoutine("C")

fun routine3LR(): AutoRoutine {
    val routine = autoFactory.newRoutine("3LR")
    val t3LS = routine.trajectory("3LS")
    val S3R = routine.trajectory("S3R")

    routine.active().onTrue(t3LS.cmd())
    t3LS.feedAtTime()
    t3LS.done().onTrue(S3R.cmd())
    S3R.scoreAtTime()

    return routine
}

fun A2R3LR(): AutoRoutine {
    val routine = autoFactory.newRoutine("A2R3LR")
    val A2R = routine.trajectory("A2R")
    val t2RS = routine.trajectory("2RS")
    val S3L = routine.trajectory("S3L")

    routine.active().onTrue(A2R.cmd())
    A2R.scoreAtTime()
    A2R.done().onTrue(t2RS.cmd())
    t2RS.feedAtTime()
    t2RS.done().onTrue(S3L.cmd())
    S3L.scoreAtTime()
    S3L.done().onTrue(routine3LR().cmd())

    routine.cmd().withName("a")
    return routine
}

fun A3LR4L(): AutoRoutine {
    val routine = autoFactory.newRoutine("A3LR4L")
    val A3L = routine.trajectory("A3L")
    val t3RS = routine.trajectory("3RS")
    val S4L = routine.trajectory("S4L")

    routine.active().onTrue(A3L.cmd())
    A3L.scoreAtTime()
    A3L.done().onTrue(routine3LR().cmd().andThen(t3RS.cmd()))
    t3RS.feedAtTime()
    t3RS.done().onTrue(S4L.cmd())
    S4L.scoreAtTime()

    return routine
}

fun B1L(): AutoRoutine {
    val routine = autoFactory.newRoutine("B1L")
    val B1L = routine.trajectory("B1L")

    routine.active().onTrue(B1L.cmd())
    B1L.scoreAtTime()

    return routine
}

fun B1R(): AutoRoutine {
    val routine = autoFactory.newRoutine("B1R")
    val B1R = routine.trajectory("B1R")

    routine.active().onTrue(B1R.cmd())
    B1R.scoreAtTime()

    return routine
}

fun B1L6RL(): AutoRoutine {
    val routine = autoFactory.newRoutine("B1L6RL")
    val t1LS = routine.trajectory("1LS")
    val S6R = routine.trajectory("S6R")
    val t6RS = routine.trajectory("6RS")
    val S6L = routine.trajectory("S6L")

    routine.active().onTrue(B1L().cmd().andThen(t1LS.cmd()))
    t1LS.feedAtTime()
    t1LS.done().onTrue(S6R.cmd())
    S6R.scoreAtTime()
    S6R.done().onTrue(t6RS.cmd())
    t6RS.feedAtTime()
    t6RS.done().onTrue(S6L.cmd())
    S6L.scoreAtTime()

    return routine
}

fun B1R2LR(): AutoRoutine {
    val routine = autoFactory.newRoutine("B1R2LR")
    val t1RS = routine.trajectory("1RS")
    val S2L = routine.trajectory("S2L")
    val t3LS = routine.trajectory("3LS")
    val S2R = routine.trajectory("S2R")

    routine.active().onTrue(B1R().cmd().andThen(t1RS.cmd()))
    t1RS.feedAtTime()
    t1RS.done().onTrue(S2L.cmd())
    S2L.scoreAtTime()
    S2L.done().onTrue(t3LS.cmd())
    t3LS.feedAtTime()
    t3LS.done().onTrue(S2R.cmd())
    S2R.scoreAtTime()

    return routine
}

fun routine5RL(): AutoRoutine {
    val routine = autoFactory.newRoutine("5RL")
    val t5RS = routine.trajectory("5RS")
    val S5L = routine.trajectory("S5L")

    routine.active().onTrue(t5RS.cmd())
    t5RS.feedAtTime()
    t5RS.done().onTrue(S5L.cmd())
    S5L.scoreAtTime()

    return routine
}

fun C5RL4R(): AutoRoutine {
    val routine = autoFactory.newRoutine("C5RL4R")
    val C5R = routine.trajectory("C5R")
    val t5LS = routine.trajectory("5LS")
    val S4L = routine.trajectory("S4L")

    routine.active().onTrue(C5R.cmd())
    C5R.scoreAtTime()
    C5R.done().onTrue(routine5RL().cmd().andThen(t5LS.cmd()))
    t5LS.feedAtTime()
    t5LS.done().onTrue(S4L.cmd())
    S4L.scoreAtTime()

    return routine
}

fun C6L5RL(): AutoRoutine {
    val routine = autoFactory.newRoutine("C6L5RL")
    val C6L = routine.trajectory("C6L")
    val t6LS = routine.trajectory("6LS")
    val S5R = routine.trajectory("S5R")

    routine.active().onTrue(C6L.cmd())
    C6L.scoreAtTime()
    C6L.done().onTrue(t6LS.cmd())
    t6LS.feedAtTime()
    t6LS.done().onTrue(S5R.cmd())
    S5R.scoreAtTime()
    S5R.done().onTrue(routine5RL().cmd())

    return routine
}

val autoRoutines =
    arrayOf(
        ALeave(),
        BLeave(),
        CLeave(),
        B1L(),
        B1R(),
        A3LR4L(),
        A2R3LR(),
        B1L6RL(),
        B1R2LR(),
        C6L5RL(),
        C5RL4R()
    )
