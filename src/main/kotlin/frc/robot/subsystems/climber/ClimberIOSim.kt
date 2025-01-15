package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.Units
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class ClimberIOSim : ClimberIO {
    override var inputs: LoggedInputClimber = LoggedInputClimber()
    var motor = TalonFXSim(2, gearRation, momentOfInertia.`in`(Units.KilogramSquareMeters), 1.0, TalonType.KRAKEN)
    var dutyCycle = DutyCycleOut(0.0)
    var positionControler =PositionVoltage(0.0)

    override fun setLatchPosition() {

    }

    override fun setPower(power:Double) {
        motor.setControl(dutyCycle.withOutput(power))
    }

    override fun setAngle() {

    }
}