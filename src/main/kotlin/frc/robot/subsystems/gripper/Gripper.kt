package frc.robot.subsystems.gripper

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Gripper(private val io: GripperIO) : SubsystemBase() {
    private val debouncer = Debouncer(DEBOUNCE_TIME.`in`(Units.Seconds), Debouncer.DebounceType.kBoth)

    @AutoLogOutput
    val hasCoral = Trigger {
        debouncer.calculate(io.inputs.sensorDistance < DISTANCE_THRESHOLD)
    }

    private fun setVoltage(voltage: Voltage): Command =
        startEnd({ io.setVoltage(voltage) }, { io.setVoltage(STOP_VOLTAGE) })

    fun intake(): Command =
        setVoltage(INTAKE_VOLTAGE).withName("Gripper/Intake")

    fun outtake(): Command =
        setVoltage(OUTTAKE_VOLTAGE).withName("Gripper/Outtake")

    fun removeAlgae(): Command =
        setVoltage(REMOVE_ALGAE_VOLTAGE).withName("Gripper/RemoveAlgae")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}
