package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.subsystems.gripper.MOMENT_OF_INERTIA

class IntakeIOSim : IntakeIO {
    override val inputs = LoggedElevatorInputs()
    private val motorPosititonRequest = PositionVoltage(0.0)
    private val motor =
        TalonFXSim(
            1,
            frc.robot.subsystems.gripper.GEAR_RATIO,
            MOMENT_OF_INERTIA,
            1.0,
            TalonType.FALCON
        )

    override fun setExtend(position: Distance) {
        val rotationalPosition =
            Units.Rotations.of(
                position.`in`(Units.Centimeter) / ROTATIONS_TO_CENTIMETER
            )
        motor.setControl(motorPosititonRequest.withPosition(rotationalPosition))
    }
    override fun setPower(percentOutput: Double) {
        motor.setControl(DutyCycleOut(percentOutput))
    }

    override fun updateInputs() {
        inputs.appliedVoltege = Units.Volts.of(motor.appliedVoltage)
        inputs.Extend =
            Units.Centimeter.of(motor.position * ROTATIONS_TO_CENTIMETER)
    }
}
