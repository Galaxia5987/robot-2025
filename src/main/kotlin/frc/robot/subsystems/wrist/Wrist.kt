package frc.robot.subsystems.wrist

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Wrist(private val io: WristIO) : SubsystemBase() {
    @AutoLogOutput
    private var atSetpoint: Trigger = Trigger {
        setpoint.angle.isNear(io.inputs.angle, AT_SETPOINT_TOLERANCE)
    }

    @AutoLogOutput private var setpoint = Angles.ZERO

    private fun setPower(power: Double): Command = runOnce {
        io.setPower(power)
    }

    private fun setAngle(angleGoal: Angles, name: String): Command =
        runOnce {
                io.setAngle(angleGoal.angle)
                setpoint = angleGoal
            }
            .withName(name)

    fun l1(): Command = setAngle(Angles.L1, "Wrist L1")
    fun l2(): Command = setAngle(Angles.L2, "Wrist L2")
    fun l3(): Command = setAngle(Angles.L3, "Wrist L3")
    fun l4(): Command = setAngle(Angles.L4, "Wrist L4")
    fun l2algae(): Command = setAngle(Angles.L2_ALGAE, "Wrist L2 Algae")
    fun l3algae(): Command = setAngle(Angles.L3_ALGAE, "Wrist L3 Algae")
    fun feeder(): Command = setAngle(Angles.FEEDER, "Wrist Feeder")
    fun retract(): Command = setAngle(Angles.ZERO, "Wrist Zero")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}
