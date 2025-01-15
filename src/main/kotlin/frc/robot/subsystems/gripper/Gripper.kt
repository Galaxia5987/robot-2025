package frc.robot.subsystems.gripper

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Gripper private constructor(private val io: GripperIO) : SubsystemBase() {
    companion object {
        @Volatile private var instance: Gripper? = null

        fun initialize(io: GripperIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Gripper(io)
                }
            }
        }

        fun getInstance(): Gripper {
            return instance
                ?: throw IllegalArgumentException(
                    "Gripper has not been initialized. Call initialize(io: GripperIO) first."
                )
        }
    }

    fun intake(): Command =
        runOnce { io.setPower(INTAKE_POWER) }.withName("GripperIntake")
    fun outtake(): Command =
        runOnce { io.setPower(OUTTAKE_POWER) }.withName("GripperOuttake")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}
