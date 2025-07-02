package frc.robot.lib

import frc.robot.CURRENT_MODE
import frc.robot.Mode

data class Gains(
    val kP: Double = 0.0,
    val kI: Double = 0.0,
    val kD: Double = 0.0,
    val kS: Double = 0.0,
    val kV: Double = 0.0,
    val kA: Double = 0.0,
    val kG: Double = 0.0
)

fun selectGainsBasedOnMode(realGains: Gains, simGains: Gains): Gains {
    return if (CURRENT_MODE == Mode.SIM) simGains else realGains
}
