package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.gripper
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
                    swerveDrive.limitlessRunVelocity(
                        ChassisSpeeds(0.8, 0.0, 0.0)
                    )
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
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[1]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("C6L")),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.hasCoral))
            .until(gripper.hasCoral.negate())
    )

private fun S5L(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[11]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5L")),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.hasCoral))
            .until(gripper.hasCoral.negate())
    )

private fun S5R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[12]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5R")),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.hasCoral))
            .until(gripper.hasCoral.negate())
    )

fun A2R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[1]!! }),
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("C6L").mirrorPath()
        ),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.hasCoral))
            .until(gripper.hasCoral.negate())
    )

private fun S3R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[11]!! }),
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("S5L").mirrorPath()
        ),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.hasCoral))
            .until(gripper.hasCoral.negate())
    )

private fun S3L(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[12]!! }),
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("S5R").mirrorPath()
        ),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.hasCoral))
            .until(gripper.hasCoral.negate())
    )

fun C6L5LR(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[1]!! }),
        C6L().until(gripper.hasCoral.negate()),
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[11]!! }),
        feederPath("6LS"),
        S5L().until(gripper.hasCoral.negate()),
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[12]!! }),
        feederPath("5LS"),
        S5R().until(gripper.hasCoral.negate())
    )

fun pathFindC6L5LR(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[1]!! }),
        alignScoreL4(),
        Commands.runOnce({ selectedFeeder = { FeederRight.flipIfNeeded() } }),
        pathFindToSelectedFeeder()
            .withDeadline(feeder(Trigger { true }, { false })),
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[11]!! }),
        alignScoreL4(),
        pathFindToSelectedFeeder()
            .withDeadline(feeder(Trigger { true }, { false })),
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[12]!! }),
        alignScoreL4()
    )

fun A2R3RL(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[1]!! }),
        A2R().until(gripper.hasCoral.negate()),
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[11]!! }),
        feederPath("6LS", true),
        S3R().until(gripper.hasCoral.negate()),
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[12]!! }),
        feederPath("5LS", true),
        S3L().until(gripper.hasCoral.negate())
    )
