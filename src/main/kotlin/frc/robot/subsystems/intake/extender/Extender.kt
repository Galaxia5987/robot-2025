package frc.robot.subsystems.intake.extender

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

class Extender(private val io: ExtenderIO) : SubsystemBase() {

    @AutoLogOutput private var setpoint = Units.Meters.zero()
    @AutoLogOutput private var setpointName = ""
    @AutoLogOutput private var error = Units.Meters.zero()
    @AutoLogOutput private var mechanism = LoggedMechanism2d(3.0, 2.0)
    private var root = mechanism.getRoot("Extender", 1.0, 1.0)
    private val ligament =
        root.append(LoggedMechanismLigament2d("ExtenderLigament", 0.569, 0.0))

    val position: () -> Distance = { io.inputs.position }

    private var finishedResettingFlag = false

    private fun setPosition(position: Positions): Command =
        runOnce {
                io.setPosition(position.position)
                setpoint = position.position
                setpointName = position.getLoggingName()
            }
            .withName("extender/setPosition")

    private fun setVoltage(voltage: Voltage): Command =
        startEnd(
                { io.setVoltage(voltage) },
                { io.setVoltage(Units.Volts.zero()) }
            )
            .withName("extender/setVoltage")

    fun extend() = setPosition(Positions.EXTENDED).withName("extender/extend")

    fun retract() =
        setPosition(Positions.RETRACTED).withName("extender/retract")

    fun reset(): Command {
        return setVoltage(RESET_VOLTAGE)
            .alongWith(runOnce { finishedResettingFlag = false })
            .until(isStuck)
            .andThen(
                runOnce { io::reset },
                runOnce { finishedResettingFlag = true }
            )
            .withName("extender/reset")
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

    @AutoLogOutput val finishedResetting = Trigger { finishedResettingFlag }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)

        error = io.inputs.position - setpoint

        ligament.length = io.inputs.position.`in`(Units.Meters)
    }
}
