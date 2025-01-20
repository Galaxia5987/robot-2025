package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*
import org.team9432.annotation.Logged

interface WristIO {
    val inputs: LoggedWristInputs

    fun setAngle(angle: Angle) {}
    fun setVoltage(voltage: Voltage) {}
    fun resetAbsoluteEncoder(angle: Angle) {}
    fun updateInputs() {}

    @Logged
    open class WristInputs {
        var angle: MutAngle = Units.Degree.zero().mutableCopy()
        var absoluteEncoderAngle: MutAngle = Units.Degree.zero().mutableCopy()
        var noOffsetAbsoluteEncoderPosition: MutAngle =
            Units.Degree.zero().mutableCopy()
        var appliedVoltage: MutVoltage = Units.Volts.zero().mutableCopy()
        var sensorDistance: MutDistance = Units.Centimeters.zero().mutableCopy()
    }
}
