package frc.robot.subsystems.wrist

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class WristIOSim : WristIO {
    override val inputs = LoggedWristInputs()

    private val motor =
        TalonFXSim(1, GEAR_RATIO, MOMENT_OF_INERTIA, 1.0, TalonType.FALCON)
    private val angleController = PIDController(1.0, 0.0, 0.0)
    private val positionControl = PositionVoltage(0.0)
    private val dutyCycle = DutyCycleOut(0.0)

    init {
        motor.setController(angleController)
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(positionControl.withPosition(angle))
    }

    override fun setPower(power: Double) {
        motor.setControl(dutyCycle.withOutput(power))
    }

    override fun resetAbsoluteEncoder(angle: Angle) {}

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.angle.mut_replace(motor.position, Units.Rotations)
        inputs.appliedVoltage.mut_replace(motor.appliedVoltage, Units.Volts)
    }
}
