package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Voltage

class GripperIOReal : GripperIO {
    override val inputs = LoggedGripperInputs()

    private val motor = TalonFX(MOTOR_PORT)
    private val control = VoltageOut(0.0).withEnableFOC(true)

    init {
        motor.configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput =
                    MotorOutputConfigs().apply {
                        NeutralMode = NeutralModeValue.Coast
                        Inverted = InvertedValue.Clockwise_Positive
                    }
                CurrentLimits =
                    CurrentLimitsConfigs().apply {
                        StatorCurrentLimitEnable = true
                        SupplyCurrentLimitEnable = true
                        StatorCurrentLimit = 40.0
                        SupplyCurrentLimit = 20.0
                    }
            }
        )
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(control.withOutput(voltage))
    }

    override fun updateInputs() {
        inputs.appliedVoltage.mut_replace(motor.motorVoltage.value)
    }
}
