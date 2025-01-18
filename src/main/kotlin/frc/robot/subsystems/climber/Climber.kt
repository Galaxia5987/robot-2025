package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.finallyDo
import org.littletonrobotics.junction.AutoLogOutput

class Climber private constructor(private val io: ClimberIO) : SubsystemBase() {
    var inputs = io.inputs

    @AutoLogOutput
    private var isTouching = Trigger {
        inputs.sensorDistance.lt(DISTANCE_THRESHOLD)
    }
    private val hasClimbed = Trigger { inputs.angle.lt(FOLDED_ANGLE) }
    private var isLatchClosed = Trigger {
        inputs.latchPosition < LATCH_TOLERANCE + CLOSE_LATCH_POSITION
    }
    private var isAttached = Trigger(isLatchClosed.and(isTouching))
    private val isFolded = Trigger { inputs.angle == FOLDED_ANGLE }

    companion object {
        @Volatile
        private var instance: Climber? = null

        fun initialize(io: ClimberIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Climber(io)
                }
            }
        }

        fun getInstance(): Climber {
            return instance
                ?: throw IllegalStateException(
                    "Climber has not been initialized. Call initialize(io: ClimberIO) first."
                )
        }
    }

    fun closeLatch(): Command =
        runOnce({ io.setLatchPosition(CLOSE_LATCH_POSITION) })

    fun openLatch(): Command =
        runOnce({ io.setLatchPosition(OPEN_LATCH_POSITION) })

    fun lock(): Command = runOnce({ io.lock() })
    fun unlock(): Command = runOnce({ io.unlock() })
    fun unfold() {
        io.setAngle(UNFOLDED_ANGLE)
    }

    fun fold() {
        io.setAngle(FOLDED_ANGLE)
    }

    fun climb(): Command =
        run { closeLatch() }
            .andThen(setAngle(FOLDED_ANGLE).onlyIf(isLatchClosed))
            .finallyDo(lock().onlyIf(isFolded))

    fun unClimb(): Command =
        run { setPower(-0.4) }
            .andThen(unlock())
            .andThen({ unfold() })
            .finallyDo(openLatch())

    private fun setAngle(angle: Angle): Command =
        runOnce({ io.setAngle(angle) })

    private fun setPower(power: Double): Command =
        runOnce({ io.setPower(power) })
}
