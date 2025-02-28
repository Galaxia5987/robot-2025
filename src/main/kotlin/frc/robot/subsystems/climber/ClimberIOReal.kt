package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Voltage

class ClimberIOReal : ClimberIO {
    override var inputs: LoggedClimberInputs = LoggedClimberInputs()

    private val motor = TalonFX(MOTOR_ID)
    private val voltageControl = VoltageOut(0.0)

    init {
        motor.configurator.apply(MOTOR_CONFIG)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageControl.withOutput(voltage))
    }

    override fun updateInput() {
        inputs.angle = motor.position.value
        inputs.appliedVoltage = motor.motorVoltage.value
        inputs.angularVelocity = motor.velocity.value
    }
}
