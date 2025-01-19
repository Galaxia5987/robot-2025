package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motorPositionRequest = PositionVoltage(0.0)
    private val dutyCycleRequest = DutyCycleOut(0.0)
    private val angleController = PIDController(0.4, 0.0, 0.5)
    private val motor = TalonFXSim(
        1,
        1.0,
        0.003,
        1.0,
        TalonType.KRAKEN_FOC
    )

    init {
        motor.setController(angleController)
    }

    override fun setHeight(height: Distance) {
        motor.setControl(
            motorPositionRequest.withPosition(
                height.timesConversionFactor(CENTIMETERS_TO_ROTATIONS)
            )
        )
    }

    override fun setPower(percentOutput: Double) {
        motor.setControl(dutyCycleRequest.withOutput(percentOutput))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.appliedVoltage = Units.Volts.of(motor.appliedVoltage)
        inputs.height =
            Units.Rotations.of(motor.position)
                .timesConversionFactor(ROTATIONS_TO_CENTIMETER)
    }
}
