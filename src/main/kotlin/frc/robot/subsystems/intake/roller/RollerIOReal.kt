package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Voltage

class RollerIOReal : RollerIO {
    override val inputs = LoggedRollerInputs()
    private val motor = TalonFX(MOTOR_ID)
    private val controlRequest = VoltageOut(0.0).withEnableFOC(true)

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
            }
        motor.configurator.apply(motorConfig)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(controlRequest.withOutput(voltage))
    }

    override fun updateInputs() {
        inputs.appliedVoltage = motor.motorVoltage.value
    }
}
