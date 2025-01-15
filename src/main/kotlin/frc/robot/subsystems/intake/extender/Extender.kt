package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput

class Extender(private val io: ExtenderIO) : SubsystemBase() {

    private fun setPosition(position: Distance) = runOnce {
        io.setPosition(position)
    }

    private fun setPower(power: Double) = runOnce {
        io.setPower(power)
    }

    fun extend() = setPosition(EXTENDED_POSITION)

    fun retract() = setPosition(RETRACTED_POSITION)

    fun reset(): Command {
        return setPower(RESET_POWER)
            .until(isStuck)
            .andThen(
                setPower(0.0),
                runOnce { io.reset() }
            )
    }

    @AutoLogOutput
    val isExtended = Trigger {
        EXTENDED_POSITION.isNear(
            io.inputs.position,
            POSITION_TOLERANCE
        )
    }

    @AutoLogOutput
    val isRetracted = Trigger {
        RETRACTED_POSITION.isNear(
            io.inputs.position,
            POSITION_TOLERANCE
        )
    }

    @AutoLogOutput
    val isComplicated = isRetracted.negate().and(isExtended.negate())

    @AutoLogOutput
    val isStuck = Trigger {
        io.inputs.motorCurrent.abs(Units.Amps) >= RESET_CURRENT_THRESHOLD.`in`(Units.Amps)
    }
}
