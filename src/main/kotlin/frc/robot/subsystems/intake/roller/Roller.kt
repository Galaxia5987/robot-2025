package frc.robot.subsystems.intake.roller

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Roller(private val io: RollerIO) : SubsystemBase() {

    private fun setVoltage(voltage: Voltage): Command =
        startEnd(
                { io.setVoltage(voltage) },
                { io.setVoltage(Units.Volts.zero()) }
            )
            .withName("roller/setVoltage")

    fun intake() = setVoltage(INTAKE_VOLTAGE).withName("roller/intake")

    fun outtake() = setVoltage(OUTTAKE_VOLTAGE).withName("roller/outtake")

    fun farOuttake() =
        setVoltage(FAR_OUTTAKE_VOLTAGE).withName("roller/farOuttake")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("intake/${this::class.simpleName}", io.inputs)
    }
}
