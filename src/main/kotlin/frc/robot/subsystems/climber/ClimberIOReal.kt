package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Voltage

class ClimberIOReal : ClimberIO {
    override var inputs: LoggedClimberInputs = LoggedClimberInputs()

    private val mainMotor = TalonFX(MOTOR_ID)
    private val voltageControl = VoltageOut(0.0)

    init {
        mainMotor.configurator.apply(MOTOR_CONFIG)
    }

    override fun setVoltage(voltage: Voltage) {
        mainMotor.setControl(voltageControl.withOutput(voltage))
    }

    override fun updateInput() {
        inputs.angle = mainMotor.position.value
        inputs.appliedVoltage = mainMotor.motorVoltage.value
        inputs.angularVelocity = mainMotor.velocity.value
    }
}
