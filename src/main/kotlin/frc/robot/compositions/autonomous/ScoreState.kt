package frc.robot.compositions.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.defer
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import frc.robot.lib.extensions.onTrue
import org.littletonrobotics.junction.Logger

// Alignment target pose, if never set, supplies the current robot position
var selectedScorePose: Pose2d = Pose2d()
var selectedScorePoseTagId: Int = 0

data class ScoreLocation(val name: String, val pose: Pose2d, val tagId: Int) {
    companion object {
        val struct = ScoreLocationStruct()
    }
}

private fun configureButtonBox() {
    val buttonBox = CommandGenericHID(4)
    (1..12).zip('A'..'L').forEach { (buttonNumber, location) ->
        val side = if (buttonNumber % 2 == 0) "L" else "R"
        buttonBox.button(buttonNumber).onTrue({ selectedScorePose = location })
    }
}


fun setPoseBasedOnButton(buttonID: Int): Command {
    val s = ScoreLocation("2L", Pose2d(), 3)
    val a = s.copy(tagId = 3)
    return defer(
        {
            runOnce({
                selectedScorePose =
                    buttonToPoseAndTagMap[buttonID]
                        ?: throw Exception("No pose for button $buttonID!!!")
                Logger.recordOutput(
                    "ScoreState/SelectedScorePose",
                    selectedScorePose
                )
            })
        },
        setOf()
    )
}
