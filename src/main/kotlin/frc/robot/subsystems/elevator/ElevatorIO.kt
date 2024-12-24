package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import org.team9432.annotation.Logged

interface ElevatorIO {
    val inputs: LoggedElevatorInputs

    fun setVoltage(voltage: Voltage) {}

    fun setHeight(position:Distance) {}

    fun setPower(percentOutput: Double) {}

    fun reset() {}

    fun updateInputs() {}

    fun updateRoutineLog(log: SysIdRoutineLog) {}


    @Logged
    open class ElevatorInputs {
        var carriageHeight: Distance = Units.Meters.of(0.0)
        var heightSetpoint: Distance = Units.Meters.of(0.0)
        var appliedVoltege: Voltage = Units.Volts.of(0.0)
    }
}
