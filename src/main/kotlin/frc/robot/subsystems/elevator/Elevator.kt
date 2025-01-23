package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class Elevator(private val io: ElevatorIO) : SubsystemBase() {
    private val mechanism = LoggedMechanism2d(3.0, 3.0)
    private val root = mechanism.getRoot("Elevator", 2.0, 0.0)
    private val elevatorLigament =
        root.append(LoggedMechanismLigament2d("ElevatorLigament", 5.0, 90.0))

    @AutoLogOutput
    private var setpointValue: Distance = Units.Millimeters.of(0.0)

    @AutoLogOutput
    private var setpointName: Positions = Positions.ZERO

    private val tuningHeight = LoggedNetworkNumber("Tuning/Elevator/heightMeters", 0.0)

    @AutoLogOutput
    private val isStuck = Trigger {
        (io.inputs.mainMotorCurrent.abs(Units.Amps) >= RESET_CURRENT_THRESHOLD.`in`(Units.Amps))
                || (io.inputs.auxMotorCurrent.abs(Units.Amps) >= RESET_CURRENT_THRESHOLD.`in`(Units.Amps))
    }

    val height: () -> Distance = { io.inputs.height }

    private fun setPosition(position: Positions): Command =
        runOnce {
            setpointValue = position.value
            setpointName = position
            io.setHeight(position.value)
        }
            .withName(position.getLoggingName())

    fun l1(): Command = setPosition(Positions.L1)
    fun l2(): Command = setPosition(Positions.L2)
    fun l3(): Command = setPosition(Positions.L3)
    fun l4(): Command = setPosition(Positions.L4)
    fun l2Algae(): Command = setPosition(Positions.L2_ALGAE)
    fun l3Algae(): Command = setPosition(Positions.L3_ALGAE)
    fun feeder(): Command = setPosition(Positions.FEEDER)
    fun zero(): Command = setPosition(Positions.ZERO)
    fun tuningPosition(): Command =
        runOnce { Positions.TUNING.value = Units.Meters.of(tuningHeight.get()) }.andThen(setPosition(Positions.TUNING))

    fun setVoltage(voltage: Voltage): Command = run {
        io.setVoltage(voltage)
    }

    fun reset(): Command = runOnce(io::reset)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this.name, io.inputs)
        Logger.recordOutput("Elevator/Mechanism2d", mechanism)
        Logger.recordOutput("Elevator/Setpoint", setpointValue)

        elevatorLigament.length = io.inputs.height.`in`(Units.Meters)
    }
}
