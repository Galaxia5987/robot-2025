package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Extender(private val io: ExtenderIO) : SubsystemBase() {

    @AutoLogOutput private var setpoint = Units.Meters.zero()
    @AutoLogOutput private var error = Units.Meters.zero()
    @AutoLogOutput private var mechanism = Mechanism2d(3.0, 2.0)
    private var root = mechanism.getRoot("Extender", 1.0, 1.0)
    private val ligament =
        root.append(MechanismLigament2d("ExtenderLigament", 0.569, 0.0))

    private var resetFlag = false

    private fun setPosition(position: Distance): Command =
        runOnce {
                io.setPosition(position)
                setpoint = position
            }
            .withName("extender/setPosition")

    private fun setPower(power: Double): Command =
        runOnce { io.setPower(power) }.withName("extender/setPower")

    fun extend() =
        setPosition(Positions.EXTENDED.position).withName("extender/extend")

    fun retract() =
        setPosition(Positions.RETRACTED.position).withName("extender/retract")

    fun reset(): Command {
        return setPower(RESET_POWER)
            .until(isStuck)
            .andThen(setPower(0.0), runOnce { io::reset })
            .finallyDo(Runnable { resetFlag = true })
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

    @AutoLogOutput
    val finishedResetting = Trigger { resetFlag }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)

        error = io.inputs.position - setpoint

        ligament.length = io.inputs.position.`in`(Units.Meters)
    }
}
