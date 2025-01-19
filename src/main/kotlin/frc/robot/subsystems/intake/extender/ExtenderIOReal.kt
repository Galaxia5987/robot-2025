package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

class ExtenderIOReal : ExtenderIO {
    override val inputs = LoggedExtenderInputs()
    private val motor = TalonFX(MOTOR_ID)
    private val positionControl = MotionMagicTorqueCurrentFOC(0.0)
    private val voltageRequest = VoltageOut(0.0)

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
}
