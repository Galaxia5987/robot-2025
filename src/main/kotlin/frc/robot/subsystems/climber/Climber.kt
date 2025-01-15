package frc.robot.subsystems.climber

import edu.wpi.first.networktables.DoubleEntry
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput

class Climber private constructor(private val io: ClimberIO) : SubsystemBase() {
    var inputs = io.inputs

    @AutoLogOutput
    private var isTouching = Trigger { inputs.sensorDistance.lt(DISTANCE_THRESHOLD) }
    private val hasClimbed = Trigger { inputs.angle.lt(FOLDED_ANGLE) }
    private var isLatchClosed = Trigger { inputs.latchPosition.lt(LATCH_TOLERANCE) }
    private var isAttached = Trigger(isLatchClosed.and(isTouching))

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
            return instance ?: throw IllegalStateException(
                "Climber has not been initialized. Call initialize(io: ClimberIO) first."
            )
        }
    }
    fun closeLatch():Command = Commands.runOnce({io.setLatchPosition(CLOSE_LATCH_POSITION)})
    fun openLatch():Command = Commands.runOnce({io.setLatchPosition(OPEN_LATCH_POSITION)})
    fun lock():Command = Commands.runOnce({io.lock()})
    fun unlock():Command = Commands.runOnce({io.unlock()})
    fun unfold(){} //TODO
    fun fold(){}   //TODO
    private fun setAngle(angle:Angle):Command = Commands.runOnce({io.setAngle(angle)})
    private fun setPower(power:Double):Command = Commands.runOnce({io.setPower(power)})

}