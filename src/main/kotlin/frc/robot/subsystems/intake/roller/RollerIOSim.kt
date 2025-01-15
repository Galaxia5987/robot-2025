package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.DutyCycleOut
import edu.wpi.first.units.Units
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class RollerIOSim : RollerIO {
    override val inputs = LoggedRollerInputs()
    private val motor =
        TalonFXSim(1, GEAR_RATIO, MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters), 1.0, TalonType.FALCON)
    private val controlRequest = DutyCycleOut(0.0)

    override fun setPower(power: Double) {
        motor.setControl(controlRequest.withOutput(power))
    }

    override fun updateInputs() {
        inputs.appliedVoltage = Units.Volts.of(motor.appliedVoltage)
    }
}
