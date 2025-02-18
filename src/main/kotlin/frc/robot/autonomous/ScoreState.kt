package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.RobotContainer
import frc.robot.lib.distanceFromPoint
import frc.robot.swerveDrive

// TODO: Add implementation for actual value
val IS_WITHIN_AUTO_SCORE_DISTANCE =
    { swerveDrive.pose.distanceFromPoint(selectedScorePose.invoke().translation) <= MAX_ALIGNMENT_DISTANCE }

val selectedScorePose: () -> Pose2d = ScorePose.`5L`.pose

// TODO: Add implementation for actual value
private val selectedHeightPose: () -> ScoreHeight = { ScoreHeight.L3 }

//private fun selectedScorePosePathfindingCommand(): Command =
//    pathFindToPose(selectedScorePose.invoke())

private fun selectedHeightCommand(outtakeTrigger: Trigger): Command =
    selectedHeightPose.invoke().command.invoke(outtakeTrigger)

fun autoScore(): Command =
    (selectedHeightCommand(Trigger { false })
        .alongWith(
            alignToPose(
                swerveDrive,
                { swerveDrive.pose },
                selectedScorePose
            )
        )).onlyIf { swerveDrive.pose.distanceFromPoint(selectedScorePose.invoke().translation) <= MAX_ALIGNMENT_DISTANCE }.alongWith(Commands.runOnce({ println("AUTO SCORE CALLED!!!") }))

fun endAutoScore(): Command =
    ConditionalCommand(
        selectedHeightCommand(Trigger { true }),
        Commands.runOnce({ swerveDrive.stop() })
    ) {
        swerveDrive.pose.distanceFromPoint(
            selectedScorePose.invoke().translation
        ) <= LINEAR_ALIGNMENT_TOLERANCE
    }
