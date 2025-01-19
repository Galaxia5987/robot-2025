package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Distance

class ElevatorIOReal : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val mainMotor = TalonFX(MAIN_ID)
    private val auxMotor = TalonFX(AUX_ID)
    private val encoder = CANcoder(ENCODER_ID)
    private val mainMotorPositionRequest = MotionMagicTorqueCurrentFOC(0.0)

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

        mainMotor.configurator.apply(motorConfig)
        auxMotor.configurator.apply(motorConfig)

        auxMotor.setControl(Follower(MAIN_ID, false))

        val encoderConfig =
            CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = ENCODER_OFSET
            }

        encoder.configurator.apply(encoderConfig)
    }

    override fun setHeight(height: Distance) {
        mainMotor.setControl(
            mainMotorPositionRequest.withPosition(
                height.timesConversionFactor(CENTIMETERS_TO_ROTATIONS)
            )
        )
    }

    override fun setPower(percentOutput: Double) {
        mainMotor.set(percentOutput)
    }

    override fun resetAbsoluteEncoder() {
        mainMotor.setPosition(0.0)
    }

    override fun updateInputs() {
        inputs.appliedVoltage = mainMotor.motorVoltage.value
        inputs.height =
            mainMotor.position.value.timesConversionFactor(
                ROTATIONS_TO_CENTIMETER
            )
        inputs.noOffsetAbsoluteEncoderPosition = encoder.absolutePosition.value
        inputs.absoluteEncoderHeight =
            encoder.position.value.timesConversionFactor(
                ROTATIONS_TO_CENTIMETER
            )
    }
}
