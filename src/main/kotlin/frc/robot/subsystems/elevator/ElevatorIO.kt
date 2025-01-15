package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ElevatorIO {
    val inputs: LoggedElevatorInputs

    fun setHeight(position: Distance) {}

    fun setPower(percentOutput: Double) {}

    fun updateInputs() {}

    @Logged
    open class ElevatorInputs {
        var Height: Distance = Units.Meters.of(0.0)
        var Setpoint: Distance = Units.Meters.of(0.0)
        var appliedVoltege: Voltage = Units.Volts.of(0.0)
    }
}
