package frc.robot.subsystems.wrist

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

val AT_SETPOINT_TOLERANCE: Angle = Units.Degrees.of(1.5)
val FORWARD_SOFT_LIMIT = Rotation2d.fromRotations(1.8)
val REVERSE_SOFT_LIMIT = Rotation2d.fromRotations(0.0)
const val ROTOR_TO_SENSOR = 1 / ((1.0 / 9.0) * (20.0 / 66.0))
const val SENSOR_TO_MECHANISM = 1 / (16.0 / 42.0)
const val GEAR_RATIO = 1 / ((1.0 / 9.0) * (20.0 / 66.0) * (16.0 / 42.0))
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.001)
val MANUAL_CONTROL_VOLTAGE: Voltage = Units.Volts.of(4.0)
val RESET_VOLTAGE: Voltage = Units.Volts.of(-4.0)
val GAINS = selectGainsBasedOnMode(Gains(kP = 150.0, kD = 8.0), Gains())

enum class Angles(val angle: Angle) {
    L1(Units.Degrees.of(20.0)),
    L2(Units.Degrees.of(0.0)),
    L3(Units.Degrees.of(153.0)),
    L3_MANUAL(Units.Degrees.of(170.0)),
    L4(Units.Degrees.of(90.0)),
    ALIGN_L2(Units.Degrees.of(120.0)),
    ALIGN_L4(Units.Degrees.of(128.0)),
    L2_ALGAE(Units.Degrees.of(80.0)),
    L3_ALGAE(Units.Degrees.of(130.0)),
    FLOOR_ALGAE(Units.Degrees.of(70.0)),
    NET(Units.Degrees.of(210.0)),
    FEEDER(Units.Degrees.of(17.0)),
    BLOCKED_FEEDER(Units.Degrees.of(25.0)),
    MAX(Units.Degrees.of(247.3)),
    ZERO(Units.Degrees.zero());

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
