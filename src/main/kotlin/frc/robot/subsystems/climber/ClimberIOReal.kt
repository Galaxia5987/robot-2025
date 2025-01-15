package frc.robot.subsystems.climber

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.StrictFollower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Servo
import frc.robot.subsystems.Port

class ClimberIOReal:ClimberIO {
    override var inputs: LoggedInputClimber = LoggedInputClimber()
    var mainMotor = TalonFX(Port.Climber.mainMotor)
    var servo1 = Servo(Port.Climber.servo1)
    var servo2 = Servo(Port.Climber.servo2)
    var auxMotor = TalonFX(Port.Climber.auxMotor)
    var dutyCycleOut = DutyCycleOut(0.0)
    var positionVoltage = PositionVoltage(0.0)

    init {
        auxMotor.setControl(StrictFollower(mainMotor.deviceID))
        listOf(auxMotor, mainMotor).forEach { it.apply {MOTOR_CONFIG} }
    }

    override fun setLatchPosition(position: Double) {
//        listOf(servo2,servo1).forEach{it.set()}
    }

    override fun setPower(power: Double) {
        mainMotor.setControl(dutyCycleOut.withOutput(power))
    }

    override fun setAngle(angle: Angle) {
        mainMotor.setControl(positionVoltage.withPosition(angle))
    }

    override fun lock() {

    }

    override fun unlock() {
    }

    override fun updateInput() {
        inputs.angle = mainMotor.position.value
        inputs.appliedVoltage = mainMotor.supplyVoltage.value
//        inputs.latchPosition =
    }
}