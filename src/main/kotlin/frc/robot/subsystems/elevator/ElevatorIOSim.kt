package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.toAngle
import frc.robot.lib.toDistance

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motorPositionRequest = PositionVoltage(0.0)
    private val dutyCycleRequest = DutyCycleOut(0.0)
    private val angleController = PIDController(0.4, 0.0, 0.5)
    private val motor =
        TalonFXSim(2, 1.0, MOMENT_OF_INERTIA, 1.0, TalonType.KRAKEN_FOC)

    init {
        motor.setController(angleController)
    }

    override fun setHeight(height: Distance) {
        motor.setControl(
            motorPositionRequest.withPosition(
                height.toAngle(SPROCKET_RADIUS, ADJUSTED_GEAR_RATIO)
            )
        )
    }

    override fun setPower(percentOutput: Double) {
        motor.setControl(dutyCycleRequest.withOutput(percentOutput))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.appliedVoltage = motor.appliedVoltage
        inputs.height =
            Units.Rotations.of(motor.position)
                .toDistance(SPROCKET_RADIUS, ADJUSTED_GEAR_RATIO)
    }
}
