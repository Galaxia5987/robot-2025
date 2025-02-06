package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

val AT_SETPOINT_TOLERANCE: Angle = Units.Degrees.of(1.0)
val MAX_ANGLE: Angle = Units.Degrees.of(101.0)
val MIN_ANGLE: Angle = Units.Degrees.of(0.08)
const val ROTOR_TO_SENSOR = 1 / ((1.0 / 25.0) * (40.0 / 40.0))
const val SENSOR_TO_MECHANISM = 1 / (16.0 / 42.0)
const val GEAR_RATIO = 1 / ((1.0 / 25.0) * (40.0 / 40.0) * (16.0 / 42.0))
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.001)
val ENCODER_OFFSET: Angle = Units.Radians.of(0.38963112012295287)

val GAINS = selectGainsBasedOnMode(
    Gains(
        kP = 75.0,
        kG = 0.2
    ),
    Gains()
)

enum class Angles(val angle: Angle) {
    L1(Units.Degrees.of(100.0)),
    L2(Units.Degrees.of(80.0)),
    L3(Units.Degrees.of(0.0)),
    L4(Units.Degrees.of(10.0)),
    L2_ALGAE(Units.Degrees.of(100.0)),
    L3_ALGAE(Units.Degrees.of(100.0)),
    FEEDER(Units.Degrees.of(45.0)),
    ZERO(Units.Degrees.zero());

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
