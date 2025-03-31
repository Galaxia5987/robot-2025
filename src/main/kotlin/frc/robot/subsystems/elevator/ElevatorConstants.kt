package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val VOLTAGE_CONTROL_KG = 0.37
val MAX_HEIGHT_LIMIT: Angle = Units.Rotations.of(27.5)
val MIN_HEIGHT_LIMIT: Angle = Units.Rotations.of(0.1337)
val AUX_MAX_HEIGHT_LIMIT: Angle = Units.Rotations.of(1.6)
val AUX_MIN_HEIGHT_LIMIT: Angle = Units.Rotations.of(-25.66)
val SETPOINT_TOLERANCE: Distance = Units.Centimeters.of(2.0)
val ALMOST_AT_SETPOINT_TOLERANCE: Distance = Units.Centimeters.of(30.0)
val RESET_VOLTAGE: Voltage = Units.Volts.of(-4.0)
val RESET_CURRENT_THRESHOLD: Current = Units.Amps.of(50.0)
const val GEAR_RATIO = (12.0 / 72.0) * (12.0 / 12.0)
const val SECOND_STAGE_RATIO = 2.0
const val ADJUSTED_GEAR_RATIO = SECOND_STAGE_RATIO * GEAR_RATIO
val MIN_KG_HEIGHT: Distance = Units.Centimeters.of(3.0)
val SPROCKET_RADIUS: Distance = Units.Millimeters.of(36.4 / 2)
val MANUAL_CONTROL_VOLTAGE: Voltage = Units.Volts.of(6.0)
val POST_L3_ALGAE_VOLTAGE: Voltage = Units.Volts.of(-3.0)

val GAINS =
    selectGainsBasedOnMode(Gains(kP = 4.0, kD = 0.3, kG = 0.0), Gains(kP = 0.4))

enum class Positions(val value: Distance) {
    L1(Units.Centimeters.of(0.0)),
    L2(Units.Centimeters.of(0.0)),
    L3(Units.Centimeters.of(15.0)),
    L4(Units.Centimeters.of(105.0)),
    ALIGN_L2(Units.Centimeters.of(0.0)),
    ALIGN_L4(Units.Centimeters.of(95.0)),
    L2_ALGAE(Units.Centimeters.of(0.0)),
    L2_ALGAE_PICKUP(Units.Centimeters.of(0.0)),
    L3_ALGAE(Units.Centimeters.of(0.0)),
    L3_ALGAE_PICKUP(Units.Centimeters.of(40.0)),
    NET(Units.Centimeters.of(107.0)),
    FEEDER(Units.Centimeters.of(20.0)),
    BLOCKED_FEEDER(Units.Centimeters.of(10.0)),
    ZERO(Units.Centimeters.zero());

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
