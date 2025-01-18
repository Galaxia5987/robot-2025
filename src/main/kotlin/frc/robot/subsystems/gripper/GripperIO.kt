package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import org.team9432.annotation.Logged

interface GripperIO {
    val inputs: LoggedGripperInputs

    fun setVoltage(power: Double) {}

    fun updateInputs() {}

    @Logged
    open class GripperInputs {
        var appliedVoltage = Units.Volts.zero()
    }
}
