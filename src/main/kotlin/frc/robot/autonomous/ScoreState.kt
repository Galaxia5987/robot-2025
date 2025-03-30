package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.lib.flipIfNeeded
import frc.robot.lib.moveBack
import frc.robot.swerveDrive
import org.littletonrobotics.junction.Logger

// Alignment target pose, if never set, supplies the current robot position
var selectedScorePose: Pair<() -> Pose2d, () -> Int> =
    Pair({ swerveDrive.pose }, { 0 })

var selectedFeeder: () -> Pose2d = { FeederRight.flipIfNeeded() }

private var lastButtonID = -1

fun setPoseBasedOnButton(buttonID: Int): Command {
    return Commands.defer(
        {
            runOnce({
                if (buttonID == lastButtonID) {
                    return@runOnce
                }

                selectedScorePose =
                    buttonToPoseAndTagMap[buttonID]
                        ?: throw Exception("No pose for button $buttonID!!!")
                Logger.recordOutput(
                    "ScoreState/SelectedScorePose",
                    selectedScorePose.first.invoke()
                )

                resetProfiledPID(
                    swerveDrive.localEstimatedPose,
                    swerveDrive.fieldOrientedSpeeds
                )
                if (justDidL2.negate().asBoolean) {
                    setGoal(
                        selectedScorePose.first
                            .invoke()
                            .moveBack(Units.Meters.of(0.1))
                    )
                } else {
                    setGoal(selectedScorePose.first.invoke())
                }

                lastButtonID = buttonID
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
