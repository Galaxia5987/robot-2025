package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.extensions.*
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class Extender(private val io: ExtenderIO) : SubsystemBase() {

    @AutoLogOutput private var setpoint = Units.Meters.zero()
    @AutoLogOutput private var setpointName = ""
    @AutoLogOutput private var error = Units.Meters.zero()
    @AutoLogOutput private var mechanism = LoggedMechanism2d(3.0, 2.0)
    private var root = mechanism.getRoot("Extender", 1.0, 1.0)
    private val ligament =
        root.append(LoggedMechanismLigament2d("ExtenderLigament", 0.569, 0.0))

    private val tuningPositionMeters =
        LoggedNetworkNumber("/Tuning/Extender/Position", 0.0)

    val position: () -> Distance = { io.inputs.position }

    private var finishedResettingFlag = false

    private fun setPosition(position: () -> Distance): Command =
        sequence(
                runOnce {
                    setpoint = position.invoke()
                    io.setPosition(setpoint)
                },
                waitUntil(atSetpoint),
                setVoltage(Units.Volts.zero())
            )
            .withName("Extender/setPosition")

    private fun setPosition(position: Positions): Command =
        runOnce {
            setpointName = position.getLoggingName()
        } then setPosition { position.position } withName "setPosition with enum"

    private fun setVoltage(voltage: Voltage): Command =
        startEnd(
                { io.setVoltage(voltage) },
                { io.setVoltage(Units.Volts.zero()) }
            )
            .withName("Extender/setVoltage")

    fun tuningPosition(): Command = runOnce {
        setpoint = tuningPositionMeters.get().m
        io.setPosition(tuningPositionMeters.get().m)
    }

    fun extend() = setPosition(Positions.EXTENDED).withName("Extender/extend")

    fun retract() =
        setPosition(Positions.RETRACTED).withName("Extender/retract")

    fun reset(resetTrigger: Trigger): Command =
        sequence(
                runOnce { io.setSoftLimits(false) }
                    .andThen(setVoltage(RESET_VOLTAGE))
                    .until(resetTrigger),
                runOnce(io::reset),
                runOnce { io.setSoftLimits(true) },
                setVoltage(Units.Volts.zero())
            )
            .withName("Extender/reset")

    fun retractTime(time: Double): Command =
        setVoltage(RESET_VOLTAGE).withTimeout(time)

    fun returnToSetpoint(): Command = run {
        atSetpoint
            .negate()
            .debounce(SAFETY_DEBOUNCE)
            .onTrue(setPosition { setpoint })
    }

    fun characterize(): Command {
        val routineForwards =
            SysIdRoutine(
                SysIdRoutine.Config(
                    5.volts / Second,
                    6.volts,
                    1.5.sec,
                    { state: State ->
                        Logger.recordOutput("Extender/state", state)
                    }
                ),
                SysIdRoutine.Mechanism(
                    { voltage: Voltage -> io.setVoltage(voltage) },
                    null,
                    this
                )
            )
        val routineBackwards =
            SysIdRoutine(
                SysIdRoutine.Config(
                    5.volts / Second,
                    4.volts,
                    1.5.sec,
                    { state: State ->
                        Logger.recordOutput("Extender/state", state)
                    }
                ),
                SysIdRoutine.Mechanism(
                    { voltage: Voltage -> io.setVoltage(voltage) },
                    null,
                    this
                )
            )
        return Commands.sequence(
                routineForwards.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0),
                routineBackwards.dynamic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1.0),
                routineForwards.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0),
                routineBackwards.quasistatic(SysIdRoutine.Direction.kReverse)
            )
            .withName("Extender/characterize")
    }

    @AutoLogOutput
    val isExtended = Trigger {
        io.inputs.position.isNear(
            Positions.EXTENDED.position,
            POSITION_TOLERANCE
        )
    }

    @AutoLogOutput
    val isRetracted = Trigger {
        io.inputs.position.isNear(
            Positions.RETRACTED.position,
            POSITION_TOLERANCE
        )
    }

    @AutoLogOutput
    val isComplicated = isRetracted.negate().and(isExtended.negate())

    @AutoLogOutput
    val isStuck = Trigger {
        io.inputs.motorCurrent.abs(Units.Amps) >=
            RESET_CURRENT_THRESHOLD.`in`(Units.Amps)
    }

    @AutoLogOutput
    private var atSetpoint = Trigger {
        io.inputs.position.isNear(setpoint, POSITION_TOLERANCE)
    }

    @AutoLogOutput
    val finishedResetting =
        Trigger { finishedResettingFlag }
            .onTrue(runOnce { io.setSoftLimits(true) })

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("Intake/${this::class.simpleName}", io.inputs)

        error = io.inputs.position - setpoint

        ligament.length = io.inputs.position.`in`(Units.Meters)
    }
}
