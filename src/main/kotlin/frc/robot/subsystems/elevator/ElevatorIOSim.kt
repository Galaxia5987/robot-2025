package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.toAngle
import frc.robot.lib.toDistance

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motorPositionRequest = PositionVoltage(0.0)
    private val voltageControl = VoltageOut(0.0)
    private val angleController = PIDController(GAINS.kP, GAINS.kI, GAINS.kD)
    private val motor = TalonFXSim(2, 1.0, 0.003, 1.0, TalonType.KRAKEN_FOC)

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

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageControl.withOutput(voltage))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.appliedVoltage = motor.appliedVoltage
        inputs.height =
            Units.Rotations.of(motor.position)
                .toDistance(SPROCKET_RADIUS, ADJUSTED_GEAR_RATIO)

        inputs.mainMotorCurrent = motor.appliedCurrent
    }
}
