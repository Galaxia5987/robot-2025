package frc.robot.subsystems.elevator

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.Gains
import frc.robot.lib.extensions.*
import frc.robot.lib.selectGainsBasedOnMode

const val VOLTAGE_CONTROL_KG = 0.37
val MAX_HEIGHT_LIMIT: Angle = 27.5.rotations
val MIN_HEIGHT_LIMIT: Angle = 0.1337.rotations
val AUX_MAX_HEIGHT_LIMIT: Angle = 1.6.rotations
val AUX_MIN_HEIGHT_LIMIT: Angle = -25.66.rotations
val SETPOINT_TOLERANCE: Distance = 2.cm
val RESET_VOLTAGE: Voltage = -4.0.volts
val RESET_CURRENT_THRESHOLD: Current = 50.amps
const val GEAR_RATIO = (12.0 / 72.0) * (12.0 / 12.0)
const val SECOND_STAGE_RATIO = 2.0
const val ADJUSTED_GEAR_RATIO = SECOND_STAGE_RATIO * GEAR_RATIO
val MIN_KG_HEIGHT: Distance = 3.cm
val SPROCKET_RADIUS: Distance = 36.4.mm / 2
val MANUAL_CONTROL_VOLTAGE: Voltage = 6.0.volts

val GAINS =
    selectGainsBasedOnMode(
        Gains(kP = 1.0, kI = 0.08, kD = 0.1, kG = 0.3),
        Gains(kP = 0.4)
    )

enum class Positions(val value: Distance) {
    L1(0.cm),
    L2(0.cm),
    L3(20.cm),
    L4(105.cm),
    ALIGN_L2(0.cm),
    ALIGN_L4(95.cm),
    L2_ALGAE(0.cm),
    L3_ALGAE(0.cm),
    FEEDER(16.cm),
    BLOCKED_FEEDER(10.cm),
    ZERO(0.cm);

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
