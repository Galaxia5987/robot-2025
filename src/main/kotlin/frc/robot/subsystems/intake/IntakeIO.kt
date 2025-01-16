package frc.robot.subsystems.intake

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface IntakeIO {
    val inputs: LoggedElevatorInputs

    fun setExtend(position: Distance) {}

    fun setPower(percentOutput: Double) {}

    fun reset() {}

    fun updateInputs() {}

    @Logged
    open class ElevatorInputs {
        var Extend: Distance = Units.Meters.of(0.0)
        var Setpoint: Distance = Units.Meters.of(0.0)
        var appliedVoltege: Voltage = Units.Volts.of(0.0)
    }
}
