package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.flipIfNeeded
import frc.robot.subsystems.l4
import frc.robot.swerveDrive

fun B1L(): Command =
    Commands.sequence(
        Commands.runOnce({swerveDrive.resetOdometry(PathPlannerPath.fromPathFile("B1R").startingDifferentialPose.flipIfNeeded())}),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1R")),
        alignCommand { l4() }.withTimeout(3.0),
        l4(Trigger { true })
    )

fun B1R(): Command =
    Commands.sequence(
        Commands.runOnce({swerveDrive.resetOdometry(PathPlannerPath.fromPathFile("B1L").startingDifferentialPose.flipIfNeeded())}),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1L")),
        alignCommand { l4() }.withTimeout(3.0),
        l4(Trigger { true })
    )
