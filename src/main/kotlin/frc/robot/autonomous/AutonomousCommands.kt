package frc.robot.autonomous

import com.ctre.phoenix6.signals.NeutralModeValue
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.elevator
import frc.robot.gripper
import frc.robot.lib.flipIfNeeded
import frc.robot.subsystems.autonomousFeeder
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.feeder
import frc.robot.swerveDrive
import frc.robot.wrist

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
                .raceWith(autonomousFeeder())
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
        wrist.skyward(),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1R")),
        alignScoreL4()
    )

fun B1RX(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[5]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("B1RX"))
            .alongWith(wrist.skyward()),
        autoScoreL4().until(gripper.autoHasCoral.negate()),
        wrist.skyward(),
        WaitCommand(0.1),
        elevator.feeder()
    )

fun `1RN`(): Command =
    Commands.sequence(
        autoAlignToReefAlgae2(),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("1RN")),
        alignAlgaeToNet().until(gripper.hasAlgae.negate())
    )

fun N2algae(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[6]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("N2"))
            .alongWith(wrist.l3algaePickup()),
        autoAlignToReefAlgae3()
    )

fun `2LN`(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("2N")),
        alignAlgaeToNet().until(gripper.hasAlgae.negate()),
        Commands.runOnce({
            swerveDrive.setNeutralMode(NeutralModeValue.Coast)
        }),
        DriveCommands.joystickDriveAtAngle(
                swerveDrive,
                { 0.7 },
                { 0.0 },
                { Rotation2d.kZero }
            )
            .withTimeout(1.8)
    )

fun C6L(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[1]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("C6L"))
            .alongWith(
                (wrist.feeder().alongWith(gripper.intake()))
                    .withTimeout(0.25)
                    .andThen(
                        WaitCommand(0.75)
                            .andThen(
                                elevator.alignL4().alongWith(wrist.alignL4())
                            )
                    )
            ),
        Commands.repeatingSequence(autoScoreL4().onlyIf(gripper.autoHasCoral))
            .until(gripper.autoHasCoral.negate())
    )

fun S5L(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[11]!! }),
        AutoBuilder.resetOdom(
            PathPlannerPath.fromPathFile("S5L").startingHolonomicPose.get()
        ),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5L"))
            .alongWith(WaitCommand(0.5).andThen(wrist.skyward())),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.autoHasCoral))
            .until(gripper.autoHasCoral.negate())
    )

fun S5R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[12]!! }),
        AutoBuilder.resetOdom(
            PathPlannerPath.fromPathFile("S5R").startingHolonomicPose.get()
        ),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5R"))
            .alongWith(WaitCommand(0.5).andThen(wrist.skyward())),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.autoHasCoral))
            .until(gripper.autoHasCoral.negate())
    )

fun A2R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[7]!! }),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("C6L").mirrorPath())
            .alongWith(
                (wrist.feeder().alongWith(gripper.intake()))
                    .withTimeout(0.25)
                    .andThen(
                        WaitCommand(0.75)
                            .andThen(
                                elevator.alignL4().alongWith(wrist.alignL4())
                            )
                    )
            ),
        Commands.repeatingSequence(autoScoreL4().onlyIf(gripper.autoHasCoral))
            .until(gripper.autoHasCoral.negate())
    )

private fun S3R(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[8]!! }),
        AutoBuilder.resetOdom(
            PathPlannerPath.fromPathFile("S5L")
                .mirrorPath()
                .startingHolonomicPose
                .get()
        ),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5L").mirrorPath())
            .alongWith(WaitCommand(0.5).andThen(wrist.skyward())),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.autoHasCoral))
            .until(gripper.autoHasCoral.negate())
    )

private fun S3L(): Command =
    Commands.sequence(
        Commands.runOnce({ selectedScorePose = buttonToPoseAndTagMap[4]!! }),
        AutoBuilder.resetOdom(
            PathPlannerPath.fromPathFile("S5R")
                .mirrorPath()
                .startingHolonomicPose
                .get()
        ),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("S5R").mirrorPath())
            .alongWith(WaitCommand(0.5).andThen(wrist.skyward())),
        Commands.repeatingSequence(alignScoreL4().onlyIf(gripper.autoHasCoral))
            .until(gripper.autoHasCoral.negate())
    )

fun B1RN2N(): Command = Commands.sequence(B1RX(), `1RN`(), N2algae(), `2LN`())

fun C6L5LR(): Command =
    Commands.sequence(
        C6L().until(gripper.autoHasCoral.negate()),
        feederPath("6LS"),
        S5L().until(gripper.autoHasCoral.negate()),
        feederPath("5LS"),
        S5R().until(gripper.autoHasCoral.negate()),
        feederPath("5RS")
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
        A2R().until(gripper.autoHasCoral.negate()),
        feederPath("6LS", true),
        S3R().until(gripper.autoHasCoral.negate()),
        feederPath("5LS", true),
        S3L().until(gripper.autoHasCoral.negate()),
        feederPath("5RS", true)
    )
