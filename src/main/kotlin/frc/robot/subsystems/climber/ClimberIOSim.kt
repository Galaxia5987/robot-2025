package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class ClimberIOSim : ClimberIO {
    override var inputs: LoggedInputClimber = LoggedInputClimber()
    var motor = TalonFXSim(2, GEAR_RATIO, MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters), 1.0, TalonType.KRAKEN)
    var dutyCycle = DutyCycleOut(0.0)
    var positionControler =PositionVoltage(0.0)

    override fun setLatchPosition(position: Distance) {

    }

    override fun lock() {
        //TODO
    }

    override fun unlock() {
        //TODO
    }

    override fun setPower(power:Double) {
        motor.setControl(dutyCycle.withOutput(power))
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(positionControler.withPosition(angle))
    }

    override fun updateInput() {
        motor.update(Timer.getFPGATimestamp())
        inputs.angle =  Units.Rotations.of(motor.position)
        inputs.appliedVoltage = Units.Volt.of(motor.appliedVoltage)
    }
}