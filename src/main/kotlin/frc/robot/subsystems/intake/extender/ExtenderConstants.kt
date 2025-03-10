package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.measure.*
import frc.robot.lib.Gains
import frc.robot.lib.extensions.*
import frc.robot.lib.selectGainsBasedOnMode

const val GEAR_RATIO = 1.0 / 3.0
val RESET_VOLTAGE: Voltage = -5.0.volts
val PINION_RADIUS: Distance = 15.22.mm
val MOMENT_OF_INERTIA: MomentOfInertia = 0.003.kg2m
val RESET_CURRENT_THRESHOLD: Current = 20.8.amps
val POSITION_TOLERANCE: Distance = 0.58.cm
val MAX_EXTENSION: Angle = 12.749.rotations
val MIN_EXTENSION: Angle = 0.0.rotations
const val SAFETY_DEBOUNCE = 1.0

val GAINS = selectGainsBasedOnMode(Gains(2.0), Gains(kP = 0.5, kD = 0.2))

enum class Positions(val position: Distance) {
    EXTENDED(34.cm),
    RETRACTED(0.cm),
    L4(36.cm);

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
