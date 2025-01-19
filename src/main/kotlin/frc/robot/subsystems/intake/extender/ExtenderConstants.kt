package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val GEAR_RATIO = 1.0 / 3.0
val RESET_VOLTAGE: Voltage = Units.Volts.of(0.0)
val PINION_RADIUS: Distance = Units.Millimeters.of(15.22)
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.003)
val RESET_CURRENT_THRESHOLD: Current = Units.Amps.of(0.0)
val POSITION_TOLERANCE: Distance = Units.Meters.of(0.0)

val GAINS = selectGainsBasedOnMode(
    Gains(),
    Gains()
)

enum class Positions(val position: Distance) {
    EXTENDED(Units.Meters.of(0.5)),
    RETRACTED(Units.Meters.of(0.0));

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
