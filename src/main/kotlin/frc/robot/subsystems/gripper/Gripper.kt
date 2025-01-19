package frc.robot.subsystems.gripper

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Gripper(private val io: GripperIO) : SubsystemBase() {
    private fun setPower(power: Double): Command = runOnce {
        io.setPower(power)
    }

    fun intake(): Command = setPower(INTAKE_POWER).withName("Gripper/Intake")
    fun outtake(): Command = setPower(OUTTAKE_POWER).withName("Gripper/Outtake")
    fun removeAlgae(): Command = setPower(REMOVE_ALGAE_POWER).withName("Gripper/RemoveAlae")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}
