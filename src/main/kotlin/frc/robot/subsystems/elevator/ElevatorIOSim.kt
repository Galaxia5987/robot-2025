package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.motors.TalonFXSim

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motorPosititonRequest = PositionVoltage(0.0)
    private val motor = TalonFXSim(
        1,
        GEAR_RATIO,
        0.3,
        1.0
    )

    override fun setHeight(position: Distance) {
        val rotationalPosition = Units.Rotations.of(position.`in`(Units.Centimeter) / ROTATIONS_TO_CENTIMETER)
        motor.setControl(motorPosititonRequest.withPosition(rotationalPosition))
    }
    override fun setPower(percentOutput: Double) {
        motor.setControl(DutyCycleOut(percentOutput))
    }

    override fun updateInputs() {
        inputs.appliedVoltege = Units.Volts.of(motor.appliedVoltage)
        inputs.height = Units.Centimeter.of(motor.position * ROTATIONS_TO_CENTIMETER)
    }
}
