package frc.robot.subsystems.climber

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.finallyDo
import org.littletonrobotics.junction.AutoLogOutput

class Climber(private val io: ClimberIO) : SubsystemBase() {
    var inputs = io.inputs

    @AutoLogOutput
    private var isTouching = Trigger {
        inputs.sensorDistance < DISTANCE_THRESHOLD
    }
    @AutoLogOutput
    private var isLatchClosed = Trigger {
        inputs.latchPosition.isNear(CLOSE_LATCH_POSITION,LATCH_TOLERANCE)
    }

    @AutoLogOutput
    private var isAttached = Trigger(isLatchClosed).and(isTouching)
    @AutoLogOutput
    private val isFolded = Trigger { inputs.angle.isNear(FOLDED_ANGLE, FOLDED_TOLERANCE)  }

    fun closeLatch(): Command =
        runOnce({ setLatchPose(CLOSE_LATCH_POSITION) })

    private fun setLatchPose(latchPose: Dimensionless) = io.setLatchPosition(latchPose)

    fun openLatch(): Command =
        runOnce({ setLatchPose(OPEN_LATCH_POSITION) })

    fun lock(): Command = runOnce({ io.lock() })
    fun unlock(): Command = runOnce({ io.unlock() })
    fun unfold() {
        io.setAngle(UNFOLDED_ANGLE)
    }

    fun fold() {
        io.setAngle(FOLDED_ANGLE)
    }

    fun climb(): Command =
        run {setLatchPose(CLOSE_LATCH_POSITION)}
            .until(isLatchClosed)
            .andThen(setAngle(FOLDED_ANGLE))
            .until(isFolded)
            .andThen(lock())

    fun unClimb(): Command =
        run {setLatchPose(CLOSE_LATCH_POSITION)}
            .andThen(unlock())
            .andThen({ unfold() })
            .andThen(openLatch())

    private fun setAngle(angle: Angle): Command =
        runOnce({ io.setAngle(angle) })

    private fun setPower(power: Double): Command =
        runOnce({ io.setPower(power) })
}
