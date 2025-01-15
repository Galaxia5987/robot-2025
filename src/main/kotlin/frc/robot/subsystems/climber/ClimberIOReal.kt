package frc.robot.subsystems.climber

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.StrictFollower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.subsystems.Port

class ClimberIOReal:ClimberIO {
    override var inputs: LoggedInputClimber = LoggedInputClimber()
    var mainMotor = TalonFX(Port.Climber.mainMotor)
    var auxMotor = TalonFX(Port.Climber.auxMotor)

    init {
        auxMotor.setControl(StrictFollower(mainMotor.deviceID))
        listOf(auxMotor, mainMotor).forEach { it.apply {MOTOR_CONFIG} }
    }

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