package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.units.measure.Angle

class WristIOReal : WristIO {
    override val inputs = LoggedWristInputs()
    private val positionControl = MotionMagicVoltage(0.0)
    private val voltageOut = VoltageOut(0.0)

    private val motor: TalonFX = TalonFX(MOTOR_PORT)
    private val absoluteEncoder = CANcoder(CANCODER_PORT)

    init {
        motor.configurator.apply(TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Feedback = FeedbackConfigs().apply {
                RotorToSensorRatio = 1.0
                SensorToMechanismRatio = GEAR_RATIO
            }
            Slot0 = Slot0Configs().apply {
                kP = 1.0
                kI = 0.0
                kD = 0.0
                kG = 0.0
                GravityType = GravityTypeValue.Arm_Cosine
                StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 20.0
                SupplyCurrentLimit = 40.0
            }
        })
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(positionControl.withPosition(angle))
    }

    override fun setVoltage(voltage: Double) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun resetAbsoluteEncoder(angle: Angle) {
        absoluteEncoder.setPosition(angle)
    }

    override fun updateInputs() {
        inputs.angle.mut_replace(motor.position.value)
        inputs.appliedVoltage.mut_replace(motor.motorVoltage.value)
        inputs.absoluteEncoderAngle.mut_replace(absoluteEncoder.position.value)
        inputs.noOffsetAbsoluteEncoderPosition.mut_replace(absoluteEncoder.position.value)
    }
}