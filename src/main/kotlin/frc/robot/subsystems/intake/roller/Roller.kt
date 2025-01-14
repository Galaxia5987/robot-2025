package frc.robot.subsystems.intake.roller

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Roller private constructor(private val io: RollerIO) : SubsystemBase() {

    companion object {
        @Volatile private var instance: Roller? = null

        fun initialize(io: RollerIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Roller(io)
                }
            }
        }

        fun getInstance(): Roller {
            return instance
                ?: throw IllegalStateException(
                    "Roller has not been initialized"
                )
        }
    }

    private fun setPower(power: Double): Command =
        Commands.runOnce({ io.setPower(power) })

    fun intake(): Command = setPower(INTAKE_POWER)

    fun outtake(): Command = setPower(OUTTAKE_POWER)

    fun farOuttake(): Command = setPower(FAR_OUTTAKE_POWER)
}
