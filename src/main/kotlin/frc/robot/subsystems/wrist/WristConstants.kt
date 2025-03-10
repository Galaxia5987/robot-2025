package frc.robot.subsystems.wrist

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.Gains
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.volts
import frc.robot.lib.selectGainsBasedOnMode

val AT_SETPOINT_TOLERANCE: Angle = 1.5.deg
val FORWARD_SOFT_LIMIT = Rotation2d.fromRotations(0.479)
val REVERSE_SOFT_LIMIT = Rotation2d.fromRotations(0.0)
const val ROTOR_TO_SENSOR = 1 / ((1.0 / 9.0) * (20.0 / 66.0))
const val SENSOR_TO_MECHANISM = 1 / (16.0 / 42.0)
const val GEAR_RATIO = 1 / ((1.0 / 9.0) * (20.0 / 66.0) * (16.0 / 42.0))
val MOMENT_OF_INERTIA: MomentOfInertia = 0.001.kg2m
val MANUAL_CONTROL_VOLTAGE: Voltage = 4.volts
val RESET_VOLTAGE: Voltage = 4.volts
val GAINS = selectGainsBasedOnMode(Gains(kP = 150.0, kD = 5.0), Gains())

enum class Angles(val angle: Angle) {
    L1(0.deg),
    L2(0.deg),
    L3(145.deg),
    L3_MANUAL(170.deg),
    L4(90.deg),
    ALIGN_L2(120.deg),
    ALIGN_L4(128.deg),
    L2_ALGAE(80.deg),
    L3_ALGAE(130.deg),
    FEEDER(17.deg),
    BLOCKED_FEEDER(25.deg),
    MAX(170.deg),
    ZERO(0.deg);

    fun getLoggingName() =
        name.split("_").joinToString(" ") {
            it.lowercase().replaceFirstChar(Char::uppercase)
        }
}
