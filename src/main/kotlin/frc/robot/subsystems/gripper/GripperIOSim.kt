package frc.robot.subsystems.gripper

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class GripperIOSim : GripperIO {
    override val inputs = LoggedElevatorInputs()
    private val motorPosititonRequest = PositionVoltage(0.0)
    private val motor =
        TalonFXSim(1, GEAR_RATIO, MOMENT_OF_INERTIA, 1.0, TalonType.FALCON)

    override fun setRotate(position: Angle) {
        val rotationalPosition =
            Units.Rotations.of(position.baseUnitMagnitude())
        motor.setControl(motorPosititonRequest.withPosition(rotationalPosition))
    }
    override fun setPower(percentOutput: Double) {
        motor.setControl(DutyCycleOut(percentOutput))
    }

    override fun updateInputs() {
        inputs.appliedVoltege = Units.Volts.of(motor.appliedVoltage)
        // inputs.Rotate = Units.Rotations.of(motor.position * ROTATIONS_TO_CENTIMETER)
    }
}
