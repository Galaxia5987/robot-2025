package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.DutyCycleOut
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class RollerIOSim : RollerIO {
    override val inputs = LoggedRollerInputs()
    private val motor =
        TalonFXSim(
            1,
            GEAR_RATIO,
            MOMENT_OF_INERTIA,
            1.0,
            TalonType.FALCON
        )
    private val controlRequest = DutyCycleOut(0.0)

    override fun setPower(power: Double) {
        motor.setControl(controlRequest.withOutput(power))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.appliedVoltage = motor.appliedVoltage
    }
}
