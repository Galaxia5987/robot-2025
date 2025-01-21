package frc.robot.subsystems.intake.extender

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

class ExtenderIOSim : ExtenderIO {
    override val inputs = LoggedExtenderInputs()
    private val motor =
        TalonFXSim(
            1,
            1.0,
            MOMENT_OF_INERTIA,
            1.0,
            TalonType.FALCON_FOC
        )
    private val positionControl = PositionVoltage(0.0)
    private val voltageControl = VoltageOut(0.0)
    private val controller = PIDController(1.0, 0.0, 0.0)

    init {
        motor.setController(controller)
    }

    override fun setPosition(position: Distance) {
        motor.setControl(
            positionControl.withPosition(
                position.toAngle(PINION_RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageControl.withOutput(voltage))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())

        inputs.position =
            Units.Rotations.of(motor.position)
                .toDistance(PINION_RADIUS, GEAR_RATIO)
        inputs.motorCurrent = motor.appliedCurrent
        inputs.appliedVoltage = motor.appliedVoltage
    }
}
