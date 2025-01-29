package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface GripperIO {
    val inputs: LoggedGripperInputs

    fun setVoltage(voltage: Voltage) {}

    fun updateInputs() {}

    @Logged
    open class GripperInputs {
        var appliedVoltage: Voltage = Units.Volts.zero().mutableCopy()
        var sensorDistance: Distance = Units.Meters.zero().mutableCopy()
    }
}
