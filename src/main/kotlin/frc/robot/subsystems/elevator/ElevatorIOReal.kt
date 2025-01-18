package frc.robot.subsystems.elevator

import ENCODER_ID
import MOTOR_ID
import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Distance

class ElevatorIOReal : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motor = TalonFX(MOTOR_ID)
    private val encoder = CANcoder(ENCODER_ID)
    private val motorPositionRequest = MotionMagicTorqueCurrentFOC(0.0)

    init {
        val motorConfig =
            TalonFXConfiguration().apply {
                MotorOutput =
                    MotorOutputConfigs().apply {
                        NeutralMode = NeutralModeValue.Brake
                        Inverted = InvertedValue.Clockwise_Positive
                    }
                Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 1.0 }
                Slot0 =
                    Slot0Configs().apply {
                        kP = GAINS.kP
                        kI = GAINS.kI
                        kD = GAINS.kD
                    }
                CurrentLimits =
                    CurrentLimitsConfigs().apply {
                        StatorCurrentLimitEnable = true
                        SupplyCurrentLimitEnable = true
                        StatorCurrentLimit = 80.0
                        SupplyCurrentLimit = 40.0
                    }
            }
        motor.configurator.apply(motorConfig)

        val encoderConfig =
            CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = ENCODER_OFSET
            }

        encoder.configurator.apply(encoderConfig)
    }

    override fun setHeight(height: Distance) {
        motor.setControl(
            motorPositionRequest.withPosition(
                height.timesConversionFactor(CENTIMETERS_TO_ROTATIONS)
            )
        )
    }

    override fun setPower(percentOutput: Double) {
        motor.set(percentOutput)
    }

    override fun resetAbsoluteEncoder() {
        motor.setPosition(0.0)
    }

    override fun updateInputs() {
        inputs.appliedVoltage = motor.motorVoltage.value
        inputs.height =
            motor.position.value.timesConversionFactor(ROTATIONS_TO_CENTIMETER)
        inputs.noOffsetAbsoluteEncoderPosition = encoder.absolutePosition.value
        inputs.absoluteEncoderHeight =
            encoder.position.value.timesConversionFactor(
                ROTATIONS_TO_CENTIMETER
            )
    }
}
