package frc.robot.subsystems.intake.roller

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface RollerIO {
    val inputs: LoggedRollerInputs

    fun setVoltage(voltage: Voltage) {}

    fun updateInputs() {}

    @Logged
    open class RollerInputs {
        var appliedVoltage: Voltage = Units.Volts.zero()
    }
}
