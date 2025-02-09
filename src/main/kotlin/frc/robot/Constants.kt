package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.lib.flipIfNeeded
import org.littletonrobotics.junction.LoggedRobot

const val LOOP_TIME = 0.02 // [s]
const val IS_TUNING_MODE = true

private val SPEAKER_POSE_BLUE = Translation2d(0.0, 5.5479442)

val SPEAKER_POSE: Translation2d
    get() = SPEAKER_POSE_BLUE.flipIfNeeded()

const val REEFMASTER_CANBUS_NAME = "reefmaster"
const val SWERVE_CANBUS_NAME = "swerveDrive"

val CURRENT_MODE: Mode
    get() =
        if (LoggedRobot.isReal()) {
            Mode.REAL
        } else {
            if (System.getenv()["isReplay"] == "true") {
                Mode.REPLAY
            } else {
                Mode.SIM
            }
        }
const val ALT_ROBORIO_SERIAL = "033E1B89"

val ROBORIO_SERIAL_NUMBER: String
    get() = System.getenv("serialnum") ?: "Sim"

val IS_RED: Boolean
    get() =
        DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red

enum class Mode {
    REAL,
    SIM,
    REPLAY
}
