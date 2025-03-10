package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.extensions.toDistance
import frc.robot.lib.extensions.toLinear

class ExtenderIOReal : ExtenderIO {
    override val inputs = LoggedExtenderInputs()
    private val motor = TalonFX(MOTOR_ID)
    private val positionControl = PositionVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    private val softLimitsConfig =
        SoftwareLimitSwitchConfigs().apply {
            ForwardSoftLimitEnable = true
            ReverseSoftLimitEnable = true
            ForwardSoftLimitThreshold = MAX_EXTENSION.`in`(Units.Rotations)
            ReverseSoftLimitThreshold = MIN_EXTENSION.`in`(Units.Rotations)
        }

    init {
        val motorConfig =
            TalonFXConfiguration().apply {
                MotorOutput =
                    MotorOutputConfigs().apply {
                        Inverted = InvertedValue.Clockwise_Positive
                        NeutralMode = NeutralModeValue.Brake
                    }

                CurrentLimits =
                    CurrentLimitsConfigs().apply {
                        StatorCurrentLimitEnable = true
                        SupplyCurrentLimitEnable = true
                        StatorCurrentLimit = 80.0
                        SupplyCurrentLimit = 40.0
                    }

                HardwareLimitSwitch =
                    HardwareLimitSwitchConfigs().apply {
                        ForwardLimitEnable = false
                        ReverseLimitEnable = false
                    }

                SoftwareLimitSwitch = softLimitsConfig

                Slot0 =
                    Slot0Configs().apply {
                        kP = GAINS.kP
                        kI = GAINS.kI
                        kD = GAINS.kD
                        kV = GAINS.kV
                        kA = GAINS.kA
                        kS = GAINS.kS
                    }
            }
        motor.configurator.apply(motorConfig)
    }

    override fun setPosition(position: Distance) {
        motor.setControl(
            positionControl.withPosition(
                position.toAngle(PINION_RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    override fun reset() {
        motor.setPosition(0.0)
    }

    override fun setSoftLimits(value: Boolean) {
        motor.configurator.apply(
            softLimitsConfig
                .withForwardSoftLimitEnable(value)
                .withReverseSoftLimitEnable(value)
        )
    }

    override fun updateInputs() {
        inputs.position =
            motor.position.value.toDistance(PINION_RADIUS, GEAR_RATIO)
        inputs.appliedVoltage = motor.motorVoltage.value
        inputs.motorCurrent = motor.supplyCurrent.value
        inputs.velocity =
            motor.velocity.value.toLinear(PINION_RADIUS, GEAR_RATIO)
    }
}
