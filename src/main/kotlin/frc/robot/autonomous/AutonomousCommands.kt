package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.flipIfNeeded
import frc.robot.subsystems.feeder
import frc.robot.swerveDrive

fun feederPath(pathName: String, mirror: Boolean = false): Command =
    AutoBuilder.followPath(
            if (mirror) PathPlannerPath.fromPathFile(pathName).mirrorPath()
            else PathPlannerPath.fromPathFile(pathName)
        )
        .andThen(
            Commands.run({
                    swerveDrive.runVelocity(ChassisSpeeds(0.8, 0.0, 0.0))
                })
                .raceWith(feeder(Trigger { true }))
        )

fun B1L(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1L")),
        Commands.runOnce({ selectedScorePose = { Reef1Left.flipIfNeeded() } }),
        alignScoreL4()
    )

fun B1R(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1R")),
        Commands.runOnce({ selectedScorePose = { Reef1Right.flipIfNeeded() } }),
        alignScoreL4()
    )

fun C6L(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("C6L")),
        Commands.runOnce({ selectedScorePose = { Reef6Left.flipIfNeeded() } }),
        autoScoreL4()
    )

private fun S5L(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5L")),
        Commands.runOnce({ selectedScorePose = { Reef5Left.flipIfNeeded() } }),
        autoScoreL4()
    )

private fun S5R(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5R")),
        Commands.runOnce({ selectedScorePose = { Reef5Right.flipIfNeeded() } }),
        autoScoreL4()
    )

fun A2R(): Command =
    Commands.sequence(
        AutoBuilder.resetOdom(
            PathPlannerPath.fromPathFile("C6L")
                .mirrorPath()
                .startingHolonomicPose
                .get()
        ),
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("C6L").mirrorPath()
        ),
        Commands.runOnce({ selectedScorePose = { Reef2Right.flipIfNeeded() } }),
        autoScoreL4()
    )

private fun S3R(): Command =
    Commands.sequence(
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("S5L").mirrorPath()
        ),
        Commands.runOnce({ selectedScorePose = { Reef3Right.flipIfNeeded() } }),
        autoScoreL4()
    )

private fun S3L(): Command =
    Commands.sequence(
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("S5R").mirrorPath()
        ),
        Commands.runOnce({ selectedScorePose = { Reef3Left.flipIfNeeded() } }),
        autoScoreL4()
    )

fun C6L5LR(): Command =
    Commands.sequence(C6L(), feederPath("6LS"), S5L(), feederPath("5LS"), S5R())

fun A2R3RL(): Command =
    Commands.sequence(
        A2R(),
        feederPath("6LS", true),
        S3R(),
        feederPath("5LS", true),
        S3L()
    )
