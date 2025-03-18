package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface GripperIO {
    val inputs: LoggedGripperInputs

    fun setVoltage(voltage: Voltage) {}

    fun updateInputs() {}

    @Logged
    open class GripperInputs {
        var appliedVoltage: MutVoltage = Units.Volts.zero().mutableCopy()
        var current: MutCurrent = Units.Amps.zero().mutableCopy()
        var sensorDistance: MutDistance = Units.Meters.zero().mutableCopy()
    }
}
