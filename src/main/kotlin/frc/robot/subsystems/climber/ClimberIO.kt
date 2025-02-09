package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ClimberIO {
    var inputs: LoggedClimberInputs
    fun setVoltage(voltage: Voltage) {}
    fun updateInput() {}

    @Logged
    open class ClimberInputs {
        var appliedVoltage: Voltage = Units.Volts.zero()
        var angle: Angle = Units.Degree.zero()
        var angularVelocity: AngularVelocity = Units.RotationsPerSecond.zero()
    }
}
