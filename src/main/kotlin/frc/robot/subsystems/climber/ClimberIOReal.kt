package frc.robot.subsystems.climber

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.StrictFollower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.AnalogInput
import frc.robot.lib.motors.LinearServo


class ClimberIOReal:ClimberIO {
    override var inputs: LoggedInputClimber = LoggedInputClimber()
    val mainMotor = TalonFX(MAIN_MOTOR_ID)
    val servo1 = LinearServo(SERVO_1_ID, 1, 1)
    val servo2 = LinearServo(SERVO_2_ID, 1, 1)
    val auxMotor = TalonFX(AUX_MOTOR_ID)
    val lockServo = TalonSRX(LOCK_MOTOR_ID)
    val dutyCycleOut = DutyCycleOut(0.0)
    val positionVoltage = PositionVoltage(0.0)
    val sensor = AnalogInput(SENSOR_ID)
    private val distanceFilter = MedianFilter(3)
    private val distance = AnalogInput(0)

    init {
        auxMotor.setControl(StrictFollower(mainMotor.deviceID))
        listOf(auxMotor, mainMotor).forEach { it.apply {MOTOR_CONFIG} }
    }

    override fun setLatchPosition(position: Double) {
        listOf(servo2, servo1).forEach { it.position = position }
    }

    override fun setPower(power: Double) {
        mainMotor.setControl(dutyCycleOut.withOutput(power))
    }

    override fun setAngle(angle: Angle) {
        mainMotor.setControl(positionVoltage.withPosition(angle))
    }

    override fun lock() {
        lockServo.set(ControlMode.Position, LOCK_ANGLE.`in`(Units.Radians))
    }

    override fun unlock() {
        lockServo.set(ControlMode.Position, UNLOCK_ANGLE.`in`(Units.Radians))

    }

    override fun updateInput() {
        inputs.angle = mainMotor.position.value
        inputs.appliedVoltage = mainMotor.supplyVoltage.value
        inputs.latchPosition = servo1.position
        var calculatedDistance = distanceFilter.calculate(4800 / (200 * distance.getVoltage() - 20.0))
        if (calculatedDistance < 0) {
            calculatedDistance = 80.0
        }

        inputs.sensorDistance = Units.Meters.of(calculatedDistance)
    }
}