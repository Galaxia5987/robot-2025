package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.lib.flipIfNeeded
import frc.robot.swerveDrive
import org.littletonrobotics.junction.Logger

// Alignment target pose, if never set, supplies the current robot position
var selectedScorePose: () -> Pose2d = { swerveDrive.pose }

fun setPoseBasedOnButton(buttonID: Int): Command {
    return Commands.defer(
        {
            runOnce({
                selectedScorePose = {
                    buttonToPoseMap[buttonID]?.flipIfNeeded()
                        ?: throw Exception("No pose for button $buttonID!!!")
                }
                Logger.recordOutput(
                    "ScoreState/SelectedScorePose",
                    selectedScorePose.invoke()
                )
            })
        },
        setOf()
    )
}
