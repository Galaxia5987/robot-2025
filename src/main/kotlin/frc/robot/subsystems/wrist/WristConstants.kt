package frc.robot.subsystems.wrist

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

val AT_SETPOINT_TOLERANCE: Angle = Units.Degrees.of(1.0)
val FORWARD_SOFT_LIMIT = Rotation2d.fromRotations(0.385)
val REVERSE_SOFT_LIMIT = Rotation2d.fromRotations(0.0)
const val ROTOR_TO_SENSOR = 1 / ((1.0 / 9.0) * (20.0 / 66.0))
const val SENSOR_TO_MECHANISM = 1 / (16.0 / 42.0)
const val GEAR_RATIO = 1 / ((1.0 / 9.0) * (20.0 / 66.0) * (16.0 / 42.0))
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.001)

val ABSOLUTE_ENCDOER_MAGNET_OFFSET = Units.Rotations.of(0.39)

val GAINS = selectGainsBasedOnMode(Gains(kP = 150.0, kD = 5.0), Gains())

enum class Angles(val angle: Angle) {
    L1(Units.Degrees.of(122.0)),
    L2(Units.Degrees.of(122.0)),
    L3(Units.Degrees.of(135.0)),
    L4(Units.Degrees.of(122.0)),
    L2_ALGAE(Units.Degrees.of(80.0)),
    L3_ALGAE(Units.Degrees.of(135.0)),
    FEEDER(Units.Degrees.of(30.0)),
    BLOCKED_FEEDER(Units.Degrees.of(42.0)),
    MAX(Units.Degrees.of(138.0)),
    ZERO(Units.Degrees.zero());

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
