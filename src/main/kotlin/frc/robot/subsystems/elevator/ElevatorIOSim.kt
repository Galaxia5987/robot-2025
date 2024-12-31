package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.motors.TalonFXSim
import kotlin.math.PI

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
        val rotationalPosition = Units.Rotations.of(position.`in`(Units.Centimeter) / (GEAR_RATIO * FIRST_STAGE_RATIO * 2 * PI * SPROCKET_RADIUS.`in`(Units.Centimeter)))
        motor.setControl(motorPosititonRequest.withPosition(rotationalPosition))
    }
    override fun setPower(percentOutput: Double) {
        motor.setControl(DutyCycleOut(percentOutput))
    }

    override fun updateInputs() {
        inputs.appliedVoltege = Units.Volts.of(motor.appliedVoltage)
        inputs.carriageHeight = Units.Centimeter.of(motor.position * GEAR_RATIO * FIRST_STAGE_RATIO * (SPROCKET_RADIUS.`in`(Units.Centimeter) * 2 * PI))
    }
}
