package frc.robot.subsystems.climber

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput

class Climber private constructor(private val io:ClimberIO):SubsystemBase(){
    var inputs = io.inputs
    @AutoLogOutput
    private var isTouching = Trigger{inputs.sensorDistance.lt(distanceThreshold)}
    private val hasClimbed = Trigger{inputs.angle.lt(FOLDED_ANGLE)}
    private var isLatchClosed = Trigger {inputs.latchPosition.lt(latchTolerance)}
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

}