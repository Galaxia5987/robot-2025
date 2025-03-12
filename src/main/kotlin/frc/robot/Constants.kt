package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.LoggedRobot

const val LOOP_TIME = 0.02 // [s]
const val IS_TUNING_MODE = true

const val SWERVE_CANBUS_NAME = "rio"

val CURRENT_MODE: Mode
    get() =
        if (LoggedRobot.isReal()) {
            Mode.REAL
        } else {
            if (System.getenv("isReplay") == "true") {
                Mode.REPLAY
            } else {
                Mode.SIM
            }
        }

val USE_MAPLE_SIM: Boolean
    get() = System.getenv("isMapleSim") == "true"

const val ALT_ROBORIO_SERIAL = "030e2d4d"

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
