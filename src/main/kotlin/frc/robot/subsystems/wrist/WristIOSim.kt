package frc.robot.subsystems.wrist

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class WristIOSim : WristIO {
    override val inputs = LoggedWristInputs()

    private val motor =
        TalonFXSim(
            1,
            GEAR_RATIO,
            MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
            1.0,
            TalonType.FALCON_FOC
        )
    private val angleController = PIDController(3.0, 0.0, 0.0)
    private val positionControl = PositionVoltage(0.0)
    private val voltageOut = VoltageOut(0.0)

    init {
        motor.setController(angleController)
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(positionControl.withPosition(angle))
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun resetAbsoluteEncoder(angle: Angle) {}

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp() * 500)
        inputs.angle = Units.Rotations.of(motor.position)
        inputs.appliedVoltage = motor.appliedVoltage
        inputs.velocity = motor.velocity
    }
}
