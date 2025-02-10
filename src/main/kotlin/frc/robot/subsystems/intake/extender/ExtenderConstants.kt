package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val GEAR_RATIO = 1.0 / 3.0
val RESET_VOLTAGE: Voltage = Units.Volts.of(-5.0)
val PINION_RADIUS: Distance = Units.Millimeters.of(15.22)
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.003)
val RESET_CURRENT_THRESHOLD: Current = Units.Amps.of(20.8)
val POSITION_TOLERANCE: Distance = Units.Centimeters.of(0.58)
val MAX_EXTENSION: Angle = Units.Rotations.of(12.749)
val MIN_EXTENSION: Angle = Units.Rotations.of(0.0)
const val SAFETY_DEBOUNCE = 1.0

val GAINS = selectGainsBasedOnMode(Gains(2.0), Gains(kP = 0.5, kD = 0.2))

enum class Positions(val position: Distance) {
    EXTENDED(Units.Meters.of(0.34)),
    RETRACTED(Units.Meters.of(0.0)),
    L4(Units.Meters.of(0.36));

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
