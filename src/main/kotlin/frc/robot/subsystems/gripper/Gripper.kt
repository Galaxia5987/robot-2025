package frc.robot.subsystems.gripper

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class Gripper(private val io: GripperIO) : SubsystemBase() {
    private val kV = 6
    var rollerAngle: Angle = Units.Rotations.zero()
    private val tuningVoltage =
        LoggedNetworkNumber("/Tuning/Gripper/Voltage", 0.0)

    @AutoLogOutput
    val hasCoral: Trigger =
        Trigger { io.inputs.sensorDistance < DISTANCE_THRESHOLD }
            .debounce(
                DEBOUNCE_TIME.`in`(Units.Seconds),
                Debouncer.DebounceType.kBoth
            )

    @AutoLogOutput
    val autoHasCoral: Trigger = Trigger {
        io.inputs.sensorDistance < DISTANCE_THRESHOLD
    }

    @AutoLogOutput
    val hasAlgae: Trigger =
        Trigger { io.inputs.current > ALGAE_CURRENT }
            .debounce(
                DEBOUNCE_TIME.`in`(Units.Seconds),
                Debouncer.DebounceType.kBoth
            )

    private fun setVoltage(voltage: Voltage): Command =
        startEnd({ io.setVoltage(voltage) }, { io.setVoltage(STOP_VOLTAGE) })

    private fun getVoltage(): Double {
        return io.inputs.appliedVoltage.`in`(Units.Volts)
    }
    fun tuningVoltage(): Command =
        setVoltage(Units.Volts.of(tuningVoltage.get()))
            .withName("Gripper/Tuning")

    fun intake(): Command =
        setVoltage(INTAKE_VOLTAGE).withName("Gripper/Intake")

    fun slowOuttake(reversed: Boolean = false): Command =
        setVoltage(
                if (reversed) -SLOW_OUTTAKE_VOLTAGE else SLOW_OUTTAKE_VOLTAGE
            )
            .withName("Gripper/SlowOuttake")

    fun outtake(reversed: Boolean = false): Command =
        setVoltage(if (reversed) -OUTTAKE_VOLTAGE else OUTTAKE_VOLTAGE)
            .withName("Gripper/Outtake")

    fun outtakeL3(): Command = setVoltage(OUTTAKE_L3).withName("Gripper/L3Outtake")

    fun fastOuttake(): Command =
        setVoltage(FAST_OUTTAKE_VOLTAGE).withName("Gripper/FastOuttake")

    fun removeAlgae(): Command =
        setVoltage(REMOVE_ALGAE_VOLTAGE).withName("Gripper/RemoveAlgae")

    fun intakeAlgae(): Command =
        Commands.runOnce({ io.setVoltage(INTAKE_ALGAE_VOLTAGE) })
            .withName("Gripper/IntakeAlgae")

    fun outtakeAlgae(): Command =
        setVoltage(OUTTAKE_ALGAE_VOLTAGE).withName("Gripper/OuttakeAlgae")

    override fun periodic() {
        rollerAngle += Units.Rotations.of(getVoltage() * kV * 0.02)
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}
