package frc.robot.autonomous

import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_RED
import frc.robot.subsystems.feeder
import frc.robot.subsystems.l4
import frc.robot.swerveDrive
import org.littletonrobotics.junction.Logger

private fun logTrajectory(
    trajectory: Trajectory<SwerveSample?>,
    isFinished: Boolean
) {
    Logger.recordOutput(
        "Odometry/Trajectory",
        *(if (IS_RED) trajectory.flipped() else trajectory)
            .samples()
            .toTypedArray()
    )
    Logger.recordOutput("Odometry/TrajectoryFinished", isFinished)
}

private val autoFactory =
    AutoFactory(
        swerveDrive::getPose,
        swerveDrive::resetOdometry,
        swerveDrive::followPath,
        true,
        swerveDrive,
        ::logTrajectory
    )

private fun AutoTrajectory.score(
    outtakeTrigger: Trigger = done(),
) =
    atTimeBeforeEnd(SCORE_TIME.`in`(Seconds))
        .onTrue(l4(outtakeTrigger))

private fun AutoTrajectory.intake(
    intakeTrigger: Trigger = done(),
) =
    atTimeBeforeEnd(INTAKE_FEEDER_TIME.`in`(Seconds))
        .onTrue(
            feeder(intakeTrigger.debounce(INTAKE_FEEDER_TIME.`in`(Seconds)))
        )

private fun leaveRoutine(startingPoint: String): AutoRoutine {
    val name = "$startingPoint Leave"
    val routine = autoFactory.newRoutine(name)
    val trajectory = routine.trajectory(name)

    routine
        .active()
        .onTrue(trajectory.resetOdometry().andThen(trajectory.cmd()))
    return routine
}

private fun createScoringSequence(
    name: String,
    trajectories: List<String>
): AutoRoutine {
    val routine = autoFactory.newRoutine(name)

    val startTrajectory = routine.trajectory(trajectories[0])
    routine
        .active()
        .onTrue(startTrajectory.resetOdometry().andThen(startTrajectory.cmd()))
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
    listOf("A", "B", "C").associate { "$it Leave" to leaveRoutine(it) } +
        mapOf(
                "B1L" to "B1L",
                "B1R" to "B1R",
                "A2R3LR" to "A2R,2RS,S3L,3LS,S3R",
                "A3LR4L" to "A3L,3LS,S3R,3RS,S4L",
                "C5RL4R" to "C5R,5RS,S5L,5LS,S4R",
                "C6L5RL" to "C6L,6LS,S5R,5RS,S5L",
                "B1R2LR" to "B1R,1RS,S2L,2LS,S2R",
                "B1L6RL" to "B1L,1LS,S6R,6RS,S6L",
            )
            .mapValues { createScoringSequence(it.key, it.value.split(",")) }
