package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.toAngle
import frc.robot.lib.toDistance
import org.littletonrobotics.junction.AutoLogOutput

class ElevatorIOReal : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val mainMotor = TalonFX(MAIN_ID)
    private val auxMotor = TalonFX(AUX_ID)
    private val encoder = CANcoder(ENCODER_ID)
    private val mainMotorPositionRequest = MotionMagicTorqueCurrentFOC(0.0)
    private val slot0Configs =
        Slot0Configs().apply {
            kP = GAINS.kP
            kI = GAINS.kI
            kD = GAINS.kD
            kG = 0.0
        }

    @AutoLogOutput
    private val kgTrigger =
        Trigger { inputs.height > MIN_KG_HEIGHT }
            .onTrue(setKG(GAINS.kG))
            .onFalse(setKG(0.0))

    init {
        val motorConfig =
            TalonFXConfiguration().apply {
                MotorOutput =
                    MotorOutputConfigs().apply {
                        NeutralMode = NeutralModeValue.Brake
                        Inverted = InvertedValue.Clockwise_Positive
                    }
                Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 1.0 }
                Slot0 = slot0Configs
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
                MagnetSensor.MagnetOffset = ENCODER_OFFSET
            }

        encoder.configurator.apply(encoderConfig)
    }

    private fun setKG(kg: Double): Command =
        runOnce({ mainMotor.configurator.apply(slot0Configs.withKG(kg)) })

    override fun setHeight(height: Distance) {
        mainMotor.setControl(
            mainMotorPositionRequest.withPosition(
                height.toAngle(SPROCKET_RADIUS, ADJUSTED_GEAR_RATIO)
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
            mainMotor.position.value.toDistance(
                SPROCKET_RADIUS,
                ADJUSTED_GEAR_RATIO
            )
        mainMotor.position.value.timesConversionFactor(ROTATIONS_TO_CENTIMETER)
        inputs.noOffsetAbsoluteEncoderPosition = encoder.absolutePosition.value
        inputs.absoluteEncoderHeight =
            encoder.position.value.toDistance(
                SPROCKET_RADIUS,
                ADJUSTED_GEAR_RATIO
            )
    }
}
