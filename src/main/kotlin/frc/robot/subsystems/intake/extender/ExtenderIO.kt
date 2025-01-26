package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ExtenderIO {
    val inputs: LoggedExtenderInputs

    fun setPosition(position: Distance) {}

    fun setVoltage(voltage: Voltage) {}

    fun reset() {}

    fun setSoftLimits(value: Boolean) {}

    fun updateInputs() {}

    @Logged
    open class ExtenderInputs {
        var appliedVoltage: Voltage = Units.Volts.zero()
        var motorCurrent: Current = Units.Amps.zero()
        var position: Distance = Units.Meters.zero()
        var sensorValue: Distance = Units.Meters.zero()
        var velocity: LinearVelocity = Units.MetersPerSecond.zero()
    }
}
