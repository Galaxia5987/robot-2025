package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ClimberIO {
    var inputs: LoggedInputClimber
    fun setLatchPosition(position: Dimensionless) {}
    fun setPower(power: Double) {}
    fun setAngle(angle: Angle) {}
    fun lock() {}
    fun unlock() {}
    fun updateInput() {}

    @Logged
    open class InputClimber {
        var appliedVoltage: Voltage = Units.Volts.zero()
        var angle: Angle = Units.Degree.zero()
        var latchPosition: Dimensionless = Units.Percent.of(0.0)
        var sensorDistance: Distance = Units.Centimeter.zero()
    }
}
