package frc.robot.subsystems.elevator

import MOTOR_ID
import com.ctre.phoenix6.hardware.TalonFX

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motor = TalonFX(MOTOR_ID)

    override fun setHeight(position: Double) {
        super.setHeight(position)
    }
    override fun setPower(percentOutput: Double) {
        super.setPower(percentOutput)
    }
    override fun reset() {}
    override fun updateInputs() {
        super.updateInputs()
    }
}
