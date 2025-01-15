package frc.robot.subsystems.intake.roller

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Roller(private val io: RollerIO) : SubsystemBase() {

    private fun setPower(power: Double) =
        Commands.runOnce({ io.setPower(power) })

    fun intake() = setPower(INTAKE_POWER)

    fun outtake() = setPower(OUTTAKE_POWER)

    fun farOuttake() = setPower(FAR_OUTTAKE_POWER)

    override fun periodic() {
        io.updateInputs()
    }
}
