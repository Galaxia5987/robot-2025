package frc.robot.subsystems.elevator

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Elevator private constructor(private val io: ElevatorIO) : SubsystemBase() {

    companion object {
        @Volatile
        private var instance: Elevator? = null

        fun initialize(io: ElevatorIO) {
            synchronized(this) {
                if (Elevator.instance == null) {
                    instance = Elevator(io)
                }
            }
        }
        fun getInstance(): Elevator {
            return Elevator.instance ?: throw IllegalStateException(
                "Elevator has not been initialized. Call initialize(io: ElevatorIO) first."
            )
        }
    }

    fun setPosition(position: Double) {
        io.setHeight(position)
    }

    fun setPower(percentOutput: Double) {
        io.setPower(percentOutput)
    }

    fun reset() {
        io.reset()
    }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("elevator", io.inputs)
    }
}
