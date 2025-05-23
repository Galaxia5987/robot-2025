package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*
import org.team9432.annotation.Logged

interface WristIO {
    val inputs: LoggedWristInputs

    fun setAngle(angle: Angle) {}
    fun setVoltage(voltage: Voltage) {}
    fun resetAbsoluteEncoder(angle: Angle) {}
    fun setSoftLimits(value: Boolean) {}
    fun updateInputs() {}

    @Logged
    open class WristInputs {
        var angle: Angle = Units.Degree.zero()
        var absoluteEncoderAngle: Angle = Units.Degree.zero()
        var noOffsetAbsoluteEncoderPosition: Angle = Units.Degree.zero()
        var appliedVoltage: Voltage = Units.Volts.zero()
        var sensorDistance: Distance = Units.Centimeters.zero()
        var velocity: AngularVelocity = Units.RotationsPerSecond.zero()
    }
}
