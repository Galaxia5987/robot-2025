package frc.robot.subsystems.climber

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Climber private constructor(private val io:ClimberIO):SubsystemBase(){
    var inputs = io.inputs

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