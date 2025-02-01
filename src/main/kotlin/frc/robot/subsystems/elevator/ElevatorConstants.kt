package frc.robot.subsystems.elevator

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.PerUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode
import kotlin.math.PI

val MAX_HEIGHT: Distance = Units.Meters.of(1.3)
val MIN_HEIGHT: Distance = Units.Meters.of(0.0)
val RESET_VOLTAGE: Voltage = Units.Volts.of(0.0)
val RESET_CURRENT_THRESHOLD: Current = Units.Amps.of(0.0)
const val GEAR_RATIO = (1.0 / 12.0) * (42.0 / 48.0)
const val FIRST_STAGE_RATIO = 2.0
const val ENCODER_OFFSET = 0.0
const val ADJUSTED_GEAR_RATIO = FIRST_STAGE_RATIO * GEAR_RATIO
val MIN_KG_HEIGHT: Distance = Units.Centimeters.of(3.0)
val SPROCKET_RADIUS: Distance = Units.Millimeters.of(36.4 / 2)
private val ROTATIONS_TO_CENTIMETERS_RATIO =
    GEAR_RATIO *
        FIRST_STAGE_RATIO *
        (SPROCKET_RADIUS.`in`(Units.Centimeter) * 2 * PI)
val ROTATIONS_TO_CENTIMETER: Measure<out PerUnit<DistanceUnit, AngleUnit>> =
    Units.Centimeters.per(Units.Rotations).of(ROTATIONS_TO_CENTIMETERS_RATIO)
val CENTIMETERS_TO_ROTATIONS: Measure<out PerUnit<AngleUnit, DistanceUnit>> =
    Units.Rotations.per(Units.Centimeter).of(1 / ROTATIONS_TO_CENTIMETERS_RATIO)

val GAINS = selectGainsBasedOnMode(Gains(kP = 1.0, kD = 1.0, kG = 1.0), Gains(kP = 0.4))

enum class Positions(val value: Distance) {
    L1(Units.Centimeters.of(0.0)),
    L2(Units.Centimeters.of(0.0)),
    L3(Units.Centimeters.of(50.0)),
    L4(Units.Centimeters.of(113.0)),
    L2_ALGAE(Units.Centimeters.of(55.0)),
    L3_ALGAE(Units.Centimeters.of(79.0)),
    FEEDER(Units.Centimeters.of(25.0)),
    ZERO(Units.Centimeters.zero());

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
