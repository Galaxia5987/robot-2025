package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage

class WristIOReal : WristIO {
    override val inputs = LoggedWristInputs()
    private val positionControl = PositionVoltage(0.0)
    private val voltageOut = VoltageOut(0.0)

    private val motor: TalonFX = TalonFX(MOTOR_PORT)
    private val absoluteEncoder = CANcoder(CANCODER_PORT)

    private val softLimits =
        SoftwareLimitSwitchConfigs().apply {
            ForwardSoftLimitEnable = true
            ReverseSoftLimitEnable = true
            ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT.rotations
            ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT.rotations
        }

    init {
        motor.configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput =
                    MotorOutputConfigs().apply {
                        NeutralMode = NeutralModeValue.Brake
                        Inverted = InvertedValue.CounterClockwise_Positive
                    }
                Feedback =
                    FeedbackConfigs().apply {
                        RotorToSensorRatio = ROTOR_TO_SENSOR
                        SensorToMechanismRatio = SENSOR_TO_MECHANISM
                        FeedbackRemoteSensorID = CANCODER_PORT
                        FeedbackSensorSource =
                            FeedbackSensorSourceValue.FusedCANcoder
                    }
                Slot0 =
                    Slot0Configs().apply {
                        kP = GAINS.kP
                        kI = GAINS.kI
                        kD = GAINS.kD
                        kG = GAINS.kG
                        GravityType = GravityTypeValue.Arm_Cosine
                        StaticFeedforwardSign =
                            StaticFeedforwardSignValue.UseClosedLoopSign
                    }
                SoftwareLimitSwitch = softLimits
                CurrentLimits =
                    CurrentLimitsConfigs().apply {
                        StatorCurrentLimitEnable = true
                        SupplyCurrentLimitEnable = true
                        StatorCurrentLimit = 20.0
                        SupplyCurrentLimit = 40.0
                    }
            }
        )

        absoluteEncoder.configurator.apply(
            CANcoderConfiguration().apply {
                MagnetSensor.SensorDirection =
                    SensorDirectionValue.Clockwise_Positive
                MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9
                MagnetSensor.MagnetOffset = 0.0
            }
        )
        absoluteEncoder.setPosition(0.0)
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(positionControl.withPosition(angle))
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun resetAbsoluteEncoder(angle: Angle) {
        absoluteEncoder.setPosition(angle)
    }

    override fun setSoftLimits(value: Boolean) {
        motor.configurator.apply(
            softLimits
                .withForwardSoftLimitEnable(value)
                .withReverseSoftLimitEnable(value)
        )
    }

    override fun updateInputs() {
        inputs.angle = motor.position.value
        inputs.appliedVoltage = motor.motorVoltage.value
        inputs.absoluteEncoderAngle = absoluteEncoder.position.value
        inputs.velocity = motor.velocity.value
    }
}
