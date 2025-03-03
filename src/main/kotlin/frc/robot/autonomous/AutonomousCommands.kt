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

fun B1L(): Command =
    Commands.sequence(
        AutoBuilder.resetOdom(
            PathPlannerPath.fromPathFile("B1L").startingHolonomicPose.get()
        ),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1L")),
        Commands.runOnce({ selectedScorePose = { Reef1Left.flipIfNeeded() } }),
        autoScoreL4()
    )

fun C6L(): Command =
    Commands.sequence(
        AutoBuilder.resetOdom(
            PathPlannerPath.fromPathFile("C6L").startingHolonomicPose.get()
        ),
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

private fun `6LS`(): Command =
    AutoBuilder.followPath(PathPlannerPath.fromPathFile("6LS"))
        .andThen(
            Commands.run({
                swerveDrive.runVelocity(
                    ChassisSpeeds(0.8, 0.0, 0.0)
                )
            })
        )
        .raceWith(feeder(Trigger { true }))

fun C6L5L(): Command =
    Commands.sequence(
        C6L(),
        `6LS`(),
        S5L()
    )
