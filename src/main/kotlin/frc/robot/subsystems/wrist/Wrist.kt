package frc.robot.subsystems.wrist

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Wrist(private val io: WristIO) : SubsystemBase() {
    @AutoLogOutput
    private var atSetpoint: Trigger = Trigger {
        setpoint.angle.isNear(io.inputs.angle, AT_SETPOINT_TOLERANCE)
    }
    private val mechanism = LoggedMechanism2d(2.0, 3.0)
    private val root = mechanism.getRoot("Wrist", 1.0, 1.0)
    private val ligament2d =
        root.append(LoggedMechanismLigament2d("WristLigament", 1.2, 0.0))

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
        Logger.recordOutput(
            "Pose",
            Pose3d(
                Translation3d(),
                Rotation3d(0.0, -io.inputs.angle.`in`(Units.Radians), 0.0)
            )
        )
        ligament2d.setAngle(io.inputs.angle.`in`(Units.Degrees))
        Logger.recordOutput("Wrist/Mechanism2d", mechanism)
        Logger.recordOutput("Wrist/Setpoint", setpoint)
    }
}
