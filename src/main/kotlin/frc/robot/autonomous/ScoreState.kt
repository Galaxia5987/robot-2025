package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.distanceFromPoint
import frc.robot.lib.flipIfNeeded
import frc.robot.swerveDrive
import org.littletonrobotics.junction.Logger

// TODO: Add implementation for actual value
val IS_WITHIN_AUTO_SCORE_DISTANCE = {
    swerveDrive.pose.distanceFromPoint(
        selectedScorePose.invoke().translation
    ) <= MAX_ALIGNMENT_DISTANCE
}

var selectedScorePose: () -> Pose2d = { Pose2d() }

private var selectedHeightPose: () -> ScoreHeight = { ScoreHeight.L3 }

var isLeft = { false }

fun setPoseBasedOnButton(buttonID: Int): Command {
    return Commands.defer({
        runOnce({
            selectedScorePose = {
                buttonToPoseMap[buttonID]?.first
                    ?: throw Exception("No pose for button $buttonID!!!")
            }
            isLeft = {
                buttonToPoseMap[buttonID]?.second
                    ?: throw Exception("isLeft not configured for $buttonID!!!")
            }
            Logger.recordOutput(
                "ScoreState/SelectedScorePose",
                selectedScorePose.invoke().flipIfNeeded()
            )

        })
    }, setOf())
}

private fun selectedHeightCommand(outtakeTrigger: Trigger): Command =
    selectedHeightPose.invoke().command.invoke(outtakeTrigger)
