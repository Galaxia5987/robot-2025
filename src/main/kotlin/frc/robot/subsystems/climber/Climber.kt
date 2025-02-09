package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Climber(private val io: ClimberIO) : SubsystemBase() {
    var inputs = io.inputs

    @AutoLogOutput private var mechanism = LoggedMechanism2d(3.0, 2.0)
    private var root = mechanism.getRoot("Climber", 1.0, 1.0)
    private val ligament =
        root.append(LoggedMechanismLigament2d("ClimberLigament", 0.27003, 90.0))

    val angle: () -> Angle = { io.inputs.angle }

    private fun setVoltage(voltage: Voltage): Command =
        startEnd(
                { io.setVoltage(voltage) },
                { io.setVoltage(Units.Volts.zero()) }
            )
            .withName("Climber/setVoltage")

    fun powerControl(power: Double): Command =
        run{setVoltage(Units.Volts.of(power * 12.0))}
            .withName("Climber/powerControl")

    fun characterize(): Command {
        val routineForwards =
            SysIdRoutine(
                SysIdRoutine.Config(
                    Units.Volt.per(Units.Second).of(5.0),
                    Units.Volt.of(6.0),
                    Units.Second.of(1.5),
                    { state: State ->
                        Logger.recordOutput("Climber/state", state)
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
                    Units.Volt.per(Units.Second).of(5.0),
                    Units.Volt.of(6.0),
                    Units.Second.of(1.5),
                    { state: State ->
                        Logger.recordOutput("Climber/state", state)
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
            .withName("Climber/characterize")
    }

    override fun periodic() {
        io.updateInput()
        Logger.processInputs(this::class.simpleName, io.inputs)

        ligament.angle = inputs.angle.`in`(Units.Degrees)
    }
}
