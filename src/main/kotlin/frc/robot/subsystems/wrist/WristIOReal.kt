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
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import frc.robot.REEFMASTER_CANBUS_NAME

class WristIOReal : WristIO {
    override val inputs = LoggedWristInputs()
    private val positionControl = PositionVoltage(0.0)
    private val voltageOut = VoltageOut(0.0)

    private val motor: TalonFX = TalonFX(MOTOR_PORT, REEFMASTER_CANBUS_NAME)
    private val absoluteEncoder =
        CANcoder(CANCODER_PORT, REEFMASTER_CANBUS_NAME)

    init {
        val encoderConfig =
            CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = ENCODER_OFFSET.`in`(Units.Rotations)
                MagnetSensor.SensorDirection =
                    SensorDirectionValue.CounterClockwise_Positive
            }

        absoluteEncoder.configurator.apply(encoderConfig)

        motor.configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput =
                    MotorOutputConfigs().apply {
                        NeutralMode = NeutralModeValue.Brake
                        Inverted = InvertedValue.Clockwise_Positive
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
                        kP = 75.0
                        kI = 0.0
                        kD = 0.0
                        kG = 0.2
                        GravityType = GravityTypeValue.Arm_Cosine
                        StaticFeedforwardSign =
                            StaticFeedforwardSignValue.UseClosedLoopSign
                    }
                SoftwareLimitSwitch =
                    SoftwareLimitSwitchConfigs().apply {
                        ForwardSoftLimitEnable = true
                        ReverseSoftLimitEnable = true
                        ForwardSoftLimitThreshold =
                            MAX_ANGLE.`in`(Units.Rotations)
                        ReverseSoftLimitThreshold =
                            MIN_ANGLE.`in`(Units.Rotations)
                    }
                CurrentLimits =
                    CurrentLimitsConfigs().apply {
                        StatorCurrentLimitEnable = true
                        SupplyCurrentLimitEnable = true
                        StatorCurrentLimit = 20.0
                        SupplyCurrentLimit = 40.0
                    }
            }
        )
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

    override fun updateInputs() {
        inputs.angle.mut_replace(motor.position.value)
        inputs.appliedVoltage.mut_replace(motor.motorVoltage.value)
        inputs.absoluteEncoderAngle.mut_replace(absoluteEncoder.position.value)
        inputs.noOffsetAbsoluteEncoderPosition.mut_replace(
            absoluteEncoder.absolutePosition.value
        )
    }
}
