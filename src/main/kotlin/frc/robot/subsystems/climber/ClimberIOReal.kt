package frc.robot.subsystems.climber

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.AnalogInput
import frc.robot.lib.motors.LinearServo

class ClimberIOReal : ClimberIO {
    override var inputs: LoggedClimberInputs = LoggedClimberInputs()

    private val mainMotor = TalonFX(MAIN_MOTOR_ID)
    private val auxMotor = TalonFX(AUX_MOTOR_ID)
    private val encoder = CANcoder(CANCODER_ID)
    private val latchServo =
        LinearServo(
            LATCH_SERVO_ID,
            1,
            1,
            LATCH_TOLERANCE.`in`(Units.Millimeters)
        )
    private val stopperMotor = TalonSRX(STOPPER_MOTOR_ID)

    private val voltageControl = VoltageOut(0.0)
    private val positionControl = PositionVoltage(0.0)

    private val sensor = AnalogInput(SENSOR_ID)
    private val distanceFilter = MedianFilter(3)

    init {
        listOf(auxMotor, mainMotor).forEach { it.apply { MOTOR_CONFIG } }
        auxMotor.setControl(Follower(mainMotor.deviceID, true))
        encoder.configurator.apply(CANCODER_CONFIG)
    }

    override fun setLatchPosition(position: Distance) {
        latchServo.position = position.`in`(Units.Millimeters)
    }

    override fun setVoltage(voltage: Voltage) {
        mainMotor.setControl(voltageControl.withOutput(voltage))
    }

    override fun setAngle(angle: Angle) {
        mainMotor.setControl(positionControl.withPosition(angle))
    }

    override fun setStopperPower(power: Double) {
        stopperMotor.set(ControlMode.PercentOutput, power)
    }

    override fun updateInput() {
        inputs.angle = mainMotor.position.value
        inputs.noOffsetEncoderPosition = encoder.absolutePosition.value
        inputs.appliedVoltage = mainMotor.supplyVoltage.value
        inputs.latchPosition = Units.Millimeters.of(latchServo.position)

        var calculatedDistance =
            distanceFilter.calculate(4800 / (200 * sensor.voltage - 20.0))
        if (calculatedDistance < 0) {
            calculatedDistance = 80.0
        }
        inputs.sensorDistance = Units.Meters.of(calculatedDistance)
        inputs.angularVelocity = mainMotor.velocity.value
    }
}
