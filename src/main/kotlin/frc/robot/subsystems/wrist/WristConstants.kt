package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle

val AT_SETPOINT_TOLERANCE: Angle = Units.Degrees.of(1.0)

enum class Angles(val angle: Angle) {
    L1(Units.Degrees.of(90.0)),
    L2(Units.Degrees.of(35.0)),
    L3(Units.Degrees.of(35.0)),
    L4(Units.Degrees.of(90.0)),
    L2_ALGAE(Units.Degrees.of(110.0)),
    L3_ALGAE(Units.Degrees.of(110.0)),
    FEEDER(Units.Degrees.of(125.0)),
    ZERO(Units.Degrees.zero());

    fun getLoggingName() = name.split("_").joinToString(" ") {
        it.lowercase().replaceFirstChar(Char::uppercase)
    }
}
