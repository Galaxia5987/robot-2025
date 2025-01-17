package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.RobotController

class GripperIOReal : GripperIO {
    override val inputs = LoggedGripperInputs()

    private val motor = TalonFX(MOTOR_PORT)
    private val control = VoltageOut(0.0).withEnableFOC(true)

    init {
        motor.configurator.apply(TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
        })
    }

    override fun setPower(power: Double) {
        motor.setControl(control.withOutput(power * RobotController.getBatteryVoltage()))
    }

    override fun updateInputs() {
        inputs.appliedVoltage = motor.motorVoltage.value
    }
}