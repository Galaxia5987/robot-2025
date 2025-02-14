package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.lib.flipIfNeeded
import org.littletonrobotics.junction.LoggedRobot

const val LOOP_TIME = 0.02 // [s]
const val IS_TUNING_MODE = true

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

val USE_MAPLE_SIM: Boolean
    get() = System.getenv()["isMapleSim"] == "true"

const val ALT_ROBORIO_SERIAL = ""

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
