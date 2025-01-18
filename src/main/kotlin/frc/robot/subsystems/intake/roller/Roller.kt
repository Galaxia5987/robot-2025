package frc.robot.subsystems.intake.roller

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Roller(private val io: RollerIO) : SubsystemBase() {

    private fun setPower(power: Double): Command = runOnce {
        io.setPower(power)
    }

    fun intake() = setPower(INTAKE_POWER)

    fun outtake() = setPower(OUTTAKE_POWER)

    fun farOuttake() = setPower(FAR_OUTTAKE_POWER)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("intake/${this::class.simpleName}", io.inputs)
    }
}
