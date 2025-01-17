package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface GripperIO {
    val inputs: LoggedElevatorInputs

    fun setRotate(position: Angle) {}

    fun setPower(percentOutput: Double) {}

    fun reset() {}

    fun updateInputs() {}

    @Logged
    open class ElevatorInputs {
        var Rotate: Angle = Units.Rotations.of(0.0)
        var Setpoint: Angle = Units.Rotations.of(0.0)
        var appliedVoltege: Voltage = Units.Volts.of(0.0)
    }
}
