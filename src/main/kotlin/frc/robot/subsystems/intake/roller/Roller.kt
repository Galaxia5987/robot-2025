package frc.robot.subsystems.intake.roller

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Roller(private val io: RollerIO) : SubsystemBase() {

    private fun setPower(power: Double): Command =
        runEnd(
            {io.setPower(power)},
            {io.setPower(0.0)}
        ).withName("roller/setPower")

    fun intake() = setPower(INTAKE_POWER).withName("roller/intake")

    fun outtake() = setPower(OUTTAKE_POWER).withName("roller/outtake")

    fun farOuttake() = setPower(FAR_OUTTAKE_POWER).withName("roller/farOuttake")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("intake/${this::class.simpleName}", io.inputs)
    }
}
