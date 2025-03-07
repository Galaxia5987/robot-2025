package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.gripper
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
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[3]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1L")),
        alignScoreL4()
    )

fun B1R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[5]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1R")),
        alignScoreL4()
    )

fun C6L(): Command =
    Commands.sequence(
        feeder(Trigger { true }, { false }),
        gripper.intake().withTimeout(0.25),
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[1]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("C6L")),
        autoScoreL4()
    )

private fun S5L(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[11]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5L")),
        autoScoreL4()
    )

private fun S5R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[12]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5R")),
        autoScoreL4()
    )

fun A2R(): Command =
    Commands.sequence(
        feeder(Trigger { true }, { false }),
        gripper.intake().withTimeout(0.25),
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[7]!! }),
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("C6L").mirrorPath()
        ),
        autoScoreL4()
    )

private fun S3R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[8]!! }),
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("S5L").mirrorPath()
        ),
        autoScoreL4()
    )

private fun S3L(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[4]!! }),
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("S5R").mirrorPath()
        ),
        autoScoreL4()
    )

fun C6L5LR(): Command =
    Commands.sequence(
        C6L(),
        Commands.either(alignScoreL4(), Commands.none(), gripper.hasCoral),
        feederPath("6LS"),
        S5L(),
        Commands.either(alignScoreL4(), Commands.none(), gripper.hasCoral),
        feederPath("5LS"),
        S5R(),
        Commands.either(alignScoreL4(), Commands.none(), gripper.hasCoral),
    )

fun A2R3RL(): Command =
    Commands.sequence(
        A2R(),
        Commands.either(alignScoreL4(), Commands.none(), gripper.hasCoral),
        feederPath("6LS", true),
        S3R(),
        Commands.either(alignScoreL4(), Commands.none(), gripper.hasCoral),
        feederPath("5LS", true),
        S3L(),
        Commands.either(alignScoreL4(), Commands.none(), gripper.hasCoral),
    )
