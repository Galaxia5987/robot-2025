package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.subsystems.gripper.STOP_VOLTAGE
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class Wrist(private val io: WristIO) : SubsystemBase() {
    @AutoLogOutput private val mechanism = LoggedMechanism2d(2.0, 3.0)
    private val root = mechanism.getRoot("Wrist", 1.0, 1.0)
    private val ligament2d =
        root.append(LoggedMechanismLigament2d("WristLigament", 1.2, 0.0))

    @AutoLogOutput
    private var setpointName: String = Angles.ZERO.getLoggingName()

    @AutoLogOutput private var setpointValue: Angle = Angles.ZERO.angle

    @AutoLogOutput
    var atSetpoint: Trigger = Trigger {
        setpointValue.isNear(io.inputs.angle, AT_SETPOINT_TOLERANCE)
    }

    private val tuningAngleDegrees =
        LoggedNetworkNumber("/Tuning/Wrist/Angle", 0.0)

    val angle: () -> Angle = { io.inputs.angle }

    fun setVoltage(voltage: Voltage): Command = runOnce {
        startEnd({ io.setVoltage(voltage) }, { io.setVoltage(STOP_VOLTAGE) })
    }

    private fun setAngle(angle: Angles): Command =
        runOnce {
                io.setAngle(angle.angle)
                setpointName = angle.getLoggingName()
                setpointValue = angle.angle
            }
            .withName("Wrist/${angle.getLoggingName()}")

    fun l1(): Command = setAngle(Angles.L1)
    fun l2(): Command = setAngle(Angles.L2)
    fun l3(): Command = setAngle(Angles.L3)
    fun l4(): Command = setAngle(Angles.L4)
    fun l2algae(): Command = setAngle(Angles.L2_ALGAE)
    fun l3algae(): Command = setAngle(Angles.L3_ALGAE)
    fun feeder(): Command = setAngle(Angles.FEEDER)
    fun blockedFeeder(): Command = setAngle(Angles.BLOCKED_FEEDER)
    fun retract(): Command = setAngle(Angles.ZERO)
    fun max(): Command = setAngle(Angles.MAX)
    fun tuningAngle(): Command =
        run {
                io.setAngle(Units.Degrees.of(tuningAngleDegrees.get()))
                setpointName = "Tuning Angle"
                setpointValue = Units.Degrees.of(tuningAngleDegrees.get())
            }
            .withName("Wrist/Tuning")

    fun characterize(): Command {
        val routineForwards =
            SysIdRoutine(
                SysIdRoutine.Config(
                    Units.Volt.per(Units.Second).of(5.0),
                    Units.Volt.of(6.0),
                    Units.Second.of(1.5),
                    { state: State ->
                        Logger.recordOutput("Wrist/state", state)
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
                        Logger.recordOutput("Wrist/state", state)
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
            .withName("Wrist/characterize")
    }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
        ligament2d.setAngle(io.inputs.angle.`in`(Units.Degrees))
    }
}
