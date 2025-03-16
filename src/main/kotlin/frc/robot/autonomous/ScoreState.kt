package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.lib.flipIfNeeded
import frc.robot.swerveDrive
import org.littletonrobotics.junction.Logger

// Alignment target pose, if never set, supplies the current robot position
var selectedScorePose: Pair<() -> Pose2d, () -> Int> =
    Pair({ swerveDrive.pose }, { 0 })

var selectedFeeder: () -> Pose2d = { FeederRight.flipIfNeeded() }

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
}

fun setFeederBasedOnAxis(axisID: Int): Command {
    return Commands.defer(
        {
            runOnce({
                selectedFeeder = {
                    if (axisID == 0) FeederRight.flipIfNeeded()
                    else FeederLeft.flipIfNeeded()
                }
                Logger.recordOutput(
                    "ScoreState/SelectedFeeder",
                    selectedFeeder.invoke()
                )
            })
        },
        setOf()
    )
}
