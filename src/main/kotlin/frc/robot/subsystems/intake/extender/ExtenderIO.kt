package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ExtenderIO {
    val inputs: LoggedExtenderInputs

    fun setPosition(position: Distance) {}

    fun setPower(power: Double) {}

    fun reset() {}

    fun updateInputs() {}

    @Logged
    open class ExtenderInputs {
        var appliedVoltage: Voltage = Units.Volts.of(0.0)
        var motorCurrent: Current = Units.Amps.of(0.0)
        var position: Distance = Units.Meters.of(0.0)
        var sensorValue: Distance = Units.Meters.of(0.0)
    }
}
