package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance

const val GEAR_RATIO = 0.0
const val RESET_POWER = 0.0
val MASS = Units.Kilogram.of(1.5)
val PINION_RADIUS = Units.Millimeters.of(15.22)
val RESET_CURRENT_THRESHOLD: Current = Units.Amps.of(0.0)
val POSITION_TOLERANCE: Distance = Units.Meters.of(0.0)

enum class Positions(val position: Distance) {
    EXTENDED(Units.Meters.of(0.0)),
    RETRACTED(Units.Meters.of(0.0));

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
