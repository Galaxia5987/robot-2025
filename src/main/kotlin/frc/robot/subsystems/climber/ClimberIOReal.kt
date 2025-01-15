package frc.robot.subsystems.climber

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance

class ClimberIOReal:ClimberIO {
    override var inputs: LoggedInputClimber = LoggedInputClimber()
//    var mainMotor = TalonFX()
    override fun setLatchPosition(position: Distance) {

    }

    override fun setPower(power: Double) {
    }

    override fun setAngle(angle: Angle) {

    }

    override fun lock() {

    }

    override fun unlock() {
        super.unlock()
    }

    override fun updateInput() {
        super.updateInput()
    }
}