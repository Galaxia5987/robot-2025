package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ClimberIO {
    var inputs:LoggedInputClimber
    fun setLatchPosition() {}
    fun setPower() {}
    fun setAngle() {}

    @Logged
    open class InputClimber {
        var appliedVoltage: Voltage = Units.Volts.zero()
        var angle: Angle = Units.Degree.zero()
        var latchPosition: Distance = Units.Centimeter.zero()
    }
}
