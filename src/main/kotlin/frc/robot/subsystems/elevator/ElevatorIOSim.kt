package frc.robot.subsystems.elevator

import MOTOR_ID
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Distance

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motor = TalonFX(MOTOR_ID)

    override fun setHeight(position: Distance) {
        val rotationalPosition=Units.Rotations.of(position.`in`(Units.Centimeter)/(GEAR_RATIO * FIRST_STAGE_RATIO * 2*PI* SPROCKET_RADIUS.`in`(Units.Centimeter)))
        motor.setControl(motorPosititonRequest.withPosition(rotationalPosition))
    }
    override fun setPower(percentOutput: Double) {
        super.setPower(percentOutput)
    }
    override fun reset() {}
    override fun updateInputs() {
        super.updateInputs()
    }
}
