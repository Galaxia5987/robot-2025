package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class ClimberIOSim : ClimberIO {
    override var inputs: LoggedClimberInputs = LoggedClimberInputs()

    private var motor =
        TalonFXSim(
            1,
            GEAR_RATIO,
            MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
            1.0,
            TalonType.KRAKEN_FOC
        )

    private var voltageControl = VoltageOut(0.0)

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageControl.withOutput(voltage))
    }

    override fun updateInput() {
        motor.update(Timer.getFPGATimestamp() * 100)
        inputs.angle = Units.Rotations.of(motor.position)
        inputs.appliedVoltage = motor.appliedVoltage
        inputs.angularVelocity = motor.velocity
    }
}
