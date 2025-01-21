package frc.robot.subsystems.gripper

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class GripperIOSim : GripperIO {
    override val inputs = LoggedGripperInputs()

    private val motor =
        TalonFXSim(
            1,
            GEAR_RATIO,
            MOMENT_OF_INERTIA,
            1.0,
            TalonType.FALCON
        )
    private val voltageOut = VoltageOut(0.0)

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.appliedVoltage.mut_replace(motor.appliedVoltage)
    }
}
