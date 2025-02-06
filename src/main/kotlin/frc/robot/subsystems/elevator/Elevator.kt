package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import java.util.function.DoubleSupplier

class Elevator(private val io: ElevatorIO) : SubsystemBase() {
    @AutoLogOutput
    private val mechanism = LoggedMechanism2d(3.0, 3.0)
    private val root = mechanism.getRoot("Elevator", 2.0, 0.0)
    private val elevatorLigament =
        root.append(LoggedMechanismLigament2d("ElevatorLigament", 5.0, 90.0))

    @AutoLogOutput
    private var setpointValue: Distance = Units.Millimeters.of(0.0)

    @AutoLogOutput
    private var setpointName: Positions = Positions.ZERO

    @AutoLogOutput
    private val isStuck = Trigger {
        maxOf(
            io.inputs.mainMotorCurrent.abs(Units.Amps),
            io.inputs.auxMotorCurrent.abs(Units.Amps)
        ) >= RESET_CURRENT_THRESHOLD.`in`(Units.Amps)
    }

    private val tuningHeight =
        LoggedNetworkNumber("/Tuning/Elevator/heightMeters", 0.0)

    val height: () -> Distance = { io.inputs.height }

    private fun setHeight(height: Positions): Command =
        run {
            setpointValue = height.value
            setpointName = height
            io.setHeight(height.value)
        }
            .withName("Elevator/setHeight ${height.getLoggingName()}")

    fun l1(): Command = setHeight(Positions.L1).withName("Elevator/L1")
    fun l2(): Command = setHeight(Positions.L2).withName("Elevator/L2")
    fun l3(): Command = setHeight(Positions.L3).withName("Elevator/L3")
    fun l4(): Command = setHeight(Positions.L4).withName("Elevator/L4")
    fun l2Algae(): Command =
        setHeight(Positions.L2_ALGAE).withName("Elevator/L2 Algae")

    fun l3Algae(): Command =
        setHeight(Positions.L3_ALGAE).withName("Elevator/L3 Algae")

    fun feeder(): Command =
        setHeight(Positions.FEEDER).withName("Elevator/Feeder")

    fun zero(): Command =
        setHeight(Positions.ZERO).withName("Elevator/Move To Zero")

    fun tuningPosition(): Command = Commands.defer({
        run {
            val height = Units.Meters.of(tuningHeight.get())
            setpointValue = height
            io.setHeight(height)
        }
            .withName("Elevator/Tuning")
    }, setOf(this))

    fun setVoltage(voltage: Voltage): Command =
        startEnd(
            { io.setVoltage(voltage) },
            { io.setVoltage(Units.Volts.zero()) }
        )
            .withName("Elevator/setVoltage")

    fun powerControl(percentOutput: DoubleSupplier): Command =
        run { io.setVoltage(Units.Volts.of(percentOutput.asDouble * 12.0 + VOLTAGE_CONTROL_KG)) }

    fun stop(): Command = setVoltage(Units.Volts.zero()).withName("Elevator/stop")

    fun reset(resetTrigger: Trigger): Command =
        Commands.sequence(
            runOnce { io.setSoftLimits(false) }
            .andThen(setVoltage(RESET_VOLTAGE)).until(resetTrigger),
            runOnce(io::reset),
            runOnce{ io.setSoftLimits(true) },
            stop()
        ).withName("Elevator/reset")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this.name, io.inputs)

        elevatorLigament.length = io.inputs.height.`in`(Units.Meters)
    }
}
