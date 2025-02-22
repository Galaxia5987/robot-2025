package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.distanceFromPoint
import frc.robot.swerveDrive
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.lang.Exception

// TODO: Add implementation for actual value
val IS_WITHIN_AUTO_SCORE_DISTANCE = {
    swerveDrive.pose.distanceFromPoint(
        selectedScorePose.invoke().translation
    ) <= MAX_ALIGNMENT_DISTANCE
}

var selectedScorePose: () -> Pose2d = ScorePose.`5L`.pose

private val buttonToPoseMap = mapOf(
    9 to Pose2d()
)

private var selectedHeightPose: () -> ScoreHeight = { ScoreHeight.L3 }

fun setPoseBasedOnButton(buttonID: Int): Command {
    return runOnce({
        selectedScorePose = { buttonToPoseMap[buttonID] ?: throw Exception("No pose for button $buttonID!!!") }
        Logger.recordOutput("ScoreState/SelectedScorePose", selectedScorePose.invoke())
    })
}


private fun selectedHeightCommand(outtakeTrigger: Trigger): Command =
    selectedHeightPose.invoke().command.invoke(outtakeTrigger)
