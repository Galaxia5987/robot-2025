package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val VOLTAGE_CONTROL_KG = 0.37
val MAX_HEIGHT_LIMIT: Distance = Units.Meters.of(0.95)
val MIN_HEIGHT_LIMIT: Distance = Units.Meters.of(0.01)
val SETPOINT_TOLERANCE: Distance = Units.Centimeters.of(7.0)
val RESET_VOLTAGE: Voltage = Units.Volts.of(-7.0)
val RESET_CURRENT_THRESHOLD: Current = Units.Amps.of(50.0)
const val GEAR_RATIO = (12.0 / 72.0) * (12.0 / 12.0)
const val SECOND_STAGE_RATIO = 2.0
const val ADJUSTED_GEAR_RATIO = SECOND_STAGE_RATIO * GEAR_RATIO
val MIN_KG_HEIGHT: Distance = Units.Centimeters.of(3.0)
val SPROCKET_RADIUS: Distance = Units.Millimeters.of(36.4 / 2)

val GAINS =
    selectGainsBasedOnMode(
        Gains(kP = 1.0, kI = 0.08, kD = 0.1, kG = 0.3),
        Gains(kP = 0.4)
    )

enum class Positions(val value: Distance) {
    L1(Units.Centimeters.of(0.0)),
    L2(Units.Centimeters.of(0.0)),
    L3(Units.Centimeters.of(30.0)),
    L4(Units.Centimeters.of(100.0)),
    L2_ALGAE(Units.Centimeters.of(0.0)),
    L3_ALGAE(Units.Centimeters.of(14.0)),
    FEEDER(Units.Centimeters.of(5.0)),
    ZERO(Units.Centimeters.zero());

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
