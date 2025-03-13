package frc.robot.compositions.autonomous

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.swerveDrive
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

// Alignment target pose, if never set, supplies the current robot position
var selectedScorePose: Pair<() -> Pose2d, () -> Int> =
    Pair({ swerveDrive.pose }, { 0 })
@AutoLogOutput(key = "Auto Alignment/Selected score pose")
var selectedScorePose = Reef1
@AutoLogOutput(key = "Auto Alignment/Is left")
var isLeft = false

data class ScoreLocation(val name: String, val pose: Pose2d, val tagId: Int)

fun setPoseBasedOnButton(buttonID: Int): Command {
    return Commands.defer(
        {
            runOnce({
                selectedScorePose =
                    buttonToPoseAndTagMap[buttonID]
                        ?: throw Exception("No pose for button $buttonID!!!")
                Logger.recordOutput(
                    "ScoreState/SelectedScorePose",
                    selectedScorePose.first.invoke()
                )
            })
        },
        setOf()
    )
    return runOnce({
        selectedScorePose = buttonToPoseMap[buttonID]?.first ?: throw Exception("No pose for button $buttonID!")

        isLeft = buttonToPoseMap[buttonID]?.second ?: throw Exception(
            "isLeft not configured for $buttonID!"
        )

        Logger.recordOutput(
            "ScoreState/SelectedScorePose", selectedScorePose.flipIfNeeded()
        )
    })
}
