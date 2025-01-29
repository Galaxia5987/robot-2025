package frc.robot.subsystems.intake.roller

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class Roller(private val io: RollerIO) : SubsystemBase() {
    private var kV = 2
    var rollerAngle: Angle = Units.Rotations.zero()
    private val tuningVoltage =
        LoggedNetworkNumber("Tuning/Roller/Voltage", 0.0)

    private fun setVoltage(voltage: Voltage): Command =
        startEnd(
                { io.setVoltage(voltage) },
                { io.setVoltage(Units.Volts.zero()) }
            )
            .withName("Roller/setVoltage")

    fun setTuningVoltage(): Command =
        setVoltage(Units.Volts.of(tuningVoltage.get()))
            .withName("Roller/Voltage")
    fun getVoltage(): Double {
        return io.inputs.appliedVoltage.baseUnitMagnitude()
    }
    fun intake() = setVoltage(INTAKE_VOLTAGE).withName("Roller/intake")

    fun outtake() = setVoltage(OUTTAKE_VOLTAGE).withName("Roller/outtake")

    fun farOuttake() =
        setVoltage(FAR_OUTTAKE_VOLTAGE).withName("Roller/farOuttake")

    override fun periodic() {
        rollerAngle += Units.Rotations.of(getVoltage() * kV * 0.02)
        io.updateInputs()
        Logger.processInputs("Intake/${this::class.simpleName}", io.inputs)
    }
}
