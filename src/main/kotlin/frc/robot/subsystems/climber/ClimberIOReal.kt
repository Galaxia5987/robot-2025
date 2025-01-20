package frc.robot.subsystems.climber

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.AnalogInput
import frc.robot.lib.motors.LinearServo


class ClimberIOReal:ClimberIO {
    override var inputs: LoggedClimberInputs = LoggedClimberInputs()

    private val mainMotor = TalonFX(MAIN_MOTOR_ID)
    private val auxMotor = TalonFX(AUX_MOTOR_ID)
    private val servo1 = LinearServo(LATCH_SERVO_ID, 1, 1)
    private val servo2 = LinearServo(FOLLOW_LATCH_SERVO_ID, 1, 1)
    private val stopperMotor = TalonSRX(STOPPER_MOTOR_ID)

    private val voltageControl = VoltageOut(0.0)
    private val positionControl = PositionVoltage(0.0)

    private val sensor = AnalogInput(SENSOR_ID)
    private val distanceFilter = MedianFilter(3)

    init {
        auxMotor.setControl(StrictFollower(mainMotor.deviceID))
        listOf(auxMotor, mainMotor).forEach { it.apply {MOTOR_CONFIG} }
    }

    override fun setLatchPosition(position: Distance) {
        listOf(servo2, servo1).forEach { it.position = position.`in`(Units.Millimeters) }
    }

    override fun setVoltage(voltage: Voltage) {
        mainMotor.setControl(voltageControl.withOutput(voltage))
    }

    override fun setAngle(angle: Angle) {
        mainMotor.setControl(positionControl.withPosition(angle))
    }

    override fun closeStopper() {
        stopperMotor.set(ControlMode.Position, LOCK_ANGLE.`in`(Units.Radians))
    }

    override fun openStopper() {
        stopperMotor.set(ControlMode.Position, UNLOCK_ANGLE.`in`(Units.Radians))

    }

    override fun updateInput() {
        inputs.angle = mainMotor.position.value
        inputs.appliedVoltage = mainMotor.supplyVoltage.value
        inputs.latchPosition = Units.Millimeters.of(servo1.position)

        var calculatedDistance = distanceFilter.calculate(4800 / (200 * sensor.voltage - 20.0))
        if (calculatedDistance < 0) {
            calculatedDistance = 80.0
        }

        inputs.sensorDistance = Units.Meters.of(calculatedDistance)
    }
}