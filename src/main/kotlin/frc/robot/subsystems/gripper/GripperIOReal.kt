package frc.robot.subsystems.gripper

import MOTOR_ID
import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.robot.CURRENT_MODE
import frc.robot.Mode

class GripperIOReal : GripperIO {
    override val inputs = LoggedElevatorInputs()
    private val motor = TalonFX(MOTOR_ID)
    private val motorPosititonRequest = PositionVoltage(0.0)

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
    }

    override fun setRotate(position: Angle) {
        inputs.Setpoint = position
        val rotationalPosition =
            Units.Rotations.of(
                position.`in`(Units.Rotations) / ROTATIONS_TO_CENTIMETER
            )
        motor.setControl(motorPosititonRequest.withPosition(rotationalPosition))
    }

    override fun setPower(percentOutput: Double) {
        motor.set(percentOutput)
    }

    override fun reset() {
        motor.setPosition(0.0)
    }
    override fun updateInputs() {
        inputs.appliedVoltege = motor.motorVoltage.value
        if (CURRENT_MODE == Mode.REAL) {

            // inputs.Rotate = Units.Rotations.of(motor.position.value.`in`(Units.Rotations) *
            // ROTATIONS_TO_CENTIMETER)
        }
    }
}
