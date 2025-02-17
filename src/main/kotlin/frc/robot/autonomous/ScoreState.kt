package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.distanceFromPoint
import frc.robot.swerveDrive

private val selectedScorePose: () -> Pose2d = ScorePose.`1L`.pose
// TODO: Add implementation for actual value

private val selectedHeightPose: () -> ScoreHeight = { ScoreHeight.L3 }
// TODO: Add implementation for actual value

private fun selectedScorePosePathfindingCommand(): Command =
    pathFindToPose(selectedScorePose.invoke())

private fun selectedHeightCommand(outtakeTrigger: Trigger): Command =
    selectedHeightPose.invoke().command.invoke(outtakeTrigger)

fun score(): Command =
    selectedScorePosePathfindingCommand()
        .andThen(
            selectedHeightCommand(Trigger { false })
                .alongWith(
                    alignToPose(
                        swerveDrive,
                        { swerveDrive.pose },
                        selectedScorePose
                    )
                )
        )

fun endScore(): Command =
    ConditionalCommand(
        selectedHeightCommand(Trigger { true }),
        Commands.runOnce({ swerveDrive.stop() })
    ) {
        swerveDrive.pose.distanceFromPoint(
            selectedScorePose.invoke().translation
        ) <= LINEAR_ALIGNMENT_TOLERANCE
    }
