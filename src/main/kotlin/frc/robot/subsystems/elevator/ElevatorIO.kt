package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ElevatorIO {
    val inputs: LoggedElevatorInputs

    fun setHeight(height: Distance) {}

    fun setPower(percentOutput: Double) {}

    fun resetAbsoluteEncoder() {}

    fun updateInputs() {}

    @Logged
    open class ElevatorInputs {
        var height: Distance = Units.Meters.zero()
        var appliedVoltage: Voltage = Units.Volts.zero()
        var limitSwitchValue = false
    }
}
