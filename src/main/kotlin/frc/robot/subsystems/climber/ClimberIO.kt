package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import org.team9432.annotation.Logged

interface ClimberIO {
    var inputs: LoggedClimberInputs
    fun setLatchPosition(position: Distance) {}
    fun setVoltage(voltage: Voltage) {}
    fun setAngle(angle: Angle) {}
    fun setStopperPower(power: Double) {}
    fun updateInput() {}
    fun updateRoutineLog(log: SysIdRoutineLog) {}

    @Logged
    open class ClimberInputs {
        var appliedVoltage: Voltage = Units.Volts.zero()
        var stopperMotorCurrent: Current = Units.Amps.zero()
        var angle: Angle = Units.Degree.zero()
        var noOffsetEncoderPosition: Angle = Units.Rotations.zero()
        var latchPosition: Distance = Units.Millimeters.zero()
        var sensorDistance: Distance = Units.Centimeter.zero()
        var angularVelocity: AngularVelocity = Units.RotationsPerSecond.zero()
    }
}
