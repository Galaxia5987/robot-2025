package frc.robot.subsystems.elevator

import MOTOR_ID
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import kotlin.math.PI

class ElevatorIOReal : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motor = TalonFX(MOTOR_ID)
    private val motorPosititonRequest = PositionVoltage(0.0)

    init {
        val motorConfig = TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Feedback = FeedbackConfigs().apply {
                RotorToSensorRatio = 1.0
//                SensorToMechanismRatio = GEAR_RATIO * FIRST_STAGE_RATIO
            }
            Slot0 = Slot0Configs().apply {
                kP = GAINS.kP
                kI = GAINS.kI
                kD = GAINS.kD
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
        }
        motor.configurator.apply(motorConfig)
    }

    override fun setHeight(position: Distance) {
        inputs.heightSetpoint=position
        val rotationalPosition= Units.Rotations.of(position.`in`(Units.Centimeter)/(GEAR_RATIO * FIRST_STAGE_RATIO * 2*PI* SPROCKET_RADIUS.`in`(Units.Centimeter)))
        motor.setControl(motorPosititonRequest.withPosition(rotationalPosition))
    }

    override fun setPower(percentOutput: Double) {
        motor.set(percentOutput)
    }

    override fun reset() {
        motor.setPosition(0.0)
    }
    override fun updateInputs() {
        inputs.appliedVoltege = motor.motorVoltage.value
        inputs.carriageHeight = Units.Centimeter.of(motor.position.value.magnitude() * GEAR_RATIO * FIRST_STAGE_RATIO * (SPROCKET_RADIUS.`in`(Units.Centimeter) * 2 * PI))
    }
}
