package frc.robot.subsystems

import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.swerveDrive

private val autoFactory =
    AutoFactory(
        swerveDrive::getPose,
        swerveDrive::setPose,
        swerveDrive::followPath,
        true,
        swerveDrive
    )

private fun AutoTrajectory.atTimeBeforeEnd(
    time: Time = Seconds.one(),
): Trigger = atTime(getRawTrajectory().totalTime - time.`in`(Seconds))

private fun AutoTrajectory.score(
    outtakeTrigger: Trigger = done(),
) = atTimeBeforeEnd().onTrue(l4(outtakeTrigger.debounce(SCORE_TIME.`in`(Seconds))))

private fun AutoTrajectory.intake(
    intakeTrigger: Trigger = done(),
) = atTimeBeforeEnd().onTrue(feeder(intakeTrigger.debounce(SCORE_TIME.`in`(Seconds))))

private fun leaveRoutine(startingPoint: String): AutoRoutine {
    val name = "$startingPoint leave"
    val routine = autoFactory.newRoutine(name)
    val trajectory = routine.trajectory(name)

    routine
        .active()
        .onTrue(trajectory.resetOdometry().andThen(trajectory.cmd()))
    return routine
}

private fun createScoringSequence(
    name: String,
    trajectories: Array<String>
): AutoRoutine {
    val routine = autoFactory.newRoutine(name)

    val startTrajectory = routine.trajectory(trajectories[0])
    routine.active().onTrue(startTrajectory.resetOdometry().andThen(startTrajectory.cmd()))
    startTrajectory.score()

    var previousTrajectory = startTrajectory

    for (trajectoryName in trajectories.drop(1)) {
        val trajectory = routine.trajectory(trajectoryName)
        previousTrajectory.done().onTrue(trajectory.cmd())
        if (trajectoryName.endsWith("S")) trajectory.intake()
        else trajectory.score()
        previousTrajectory = trajectory
    }

    return routine
}

val autoRoutines: Map<String, AutoRoutine> =
    mapOf(
        "A leave" to leaveRoutine("A"),
        "B leave" to leaveRoutine("B"),
        "C leave" to leaveRoutine("C")
    ).plus(
            mapOf(
                "B1L" to arrayOf("B1L"),
                "B1R" to arrayOf("B1R"),
                "A2R3LR" to arrayOf("A2R", "2RS", "S3L"),
                "A2R4LR" to arrayOf("A2R", "3RS", "S4L"),
                "C5RL4R" to arrayOf("C5R", "5LS", "S4L"),
                "C6L5RL" to arrayOf("C6L", "6LS", "S5R"),
                "B1R2LR" to arrayOf("1RS", "S2L", "3LS", "S2R"),
                "B1L6RL" to arrayOf("1LS", "S6R", "6RS", "S6L")
            )
                .mapValues { createScoringSequence(it.key, it.value) })