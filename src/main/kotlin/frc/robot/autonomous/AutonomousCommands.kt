package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.flipIfNeeded

fun B1L(): Command =
    Commands.sequence(
        AutoBuilder.resetOdom(PathPlannerPath.fromPathFile("B1L").startingHolonomicPose.get()),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1L")),
        Commands.runOnce({
            selectedScorePose = {Reef1Left.flipIfNeeded()}
        }),
        autoScoreL4()
    )
