package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia

val AT_SETPOINT_TOLERANCE: Angle = Units.Degrees.of(1.0)
val MAX_ANGLE: Angle = Units.Rotations.of(0.0)
val MIN_ANGLE: Angle = Units.Rotations.of(0.0)
const val GEAR_RATIO = 1.0 // TODO: Calibrate
const val ROTOR_TO_SENSOR = 1.0
const val SENSOR_TO_MECHANISM = 1.0
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.001)
const val ENCODER_OFFSET = 0.0

enum class Angles(val angle: Angle) {
    L1(Units.Degrees.of(10.0)),
    L2(Units.Degrees.of(30.0)),
    L3(Units.Degrees.of(35.0)),
    L4(Units.Degrees.of(30.0)),
    L2_ALGAE(Units.Degrees.of(110.0)),
    L3_ALGAE(Units.Degrees.of(110.0)),
    FEEDER(Units.Degrees.of(55.0)),
    ZERO(Units.Degrees.zero());

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
