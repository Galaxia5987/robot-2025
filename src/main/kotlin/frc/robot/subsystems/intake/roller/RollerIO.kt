package frc.robot.subsystems.intake.roller

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface RollerIO {
    val inputs: LoggedRollerInputs

    fun setPower(power: Double) {}

    fun updateInputs() {}

    @Logged
    open class RollerInputs {
        var appliedVoltage: Voltage = Units.Volts.of(0.0)
    }
}
