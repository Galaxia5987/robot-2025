package frc.robot.subsystems.wrist

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Wrist(private val io: WristIO) : SubsystemBase() {
    @AutoLogOutput private val mechanism = LoggedMechanism2d(2.0, 3.0)
    private val root = mechanism.getRoot("Wrist", 1.0, 1.0)
    private val ligament2d =
        root.append(LoggedMechanismLigament2d("WristLigament", 1.2, 0.0))

    @AutoLogOutput private var setpointName: Angles = Angles.ZERO
    @AutoLogOutput private var setpointValue: Angle = Angles.ZERO.angle

    @AutoLogOutput
    private var atSetpoint: Trigger = Trigger {
        setpointValue.isNear(io.inputs.angle, AT_SETPOINT_TOLERANCE)
    }

    private fun setVoltage(voltage: Voltage): Command = runOnce {
        io.setVoltage(voltage)
    }

    private fun setAngle(angle: Angles): Command =
        runOnce {
                io.setAngle(angle.angle)
                setpointName = angle
                setpointValue = angle.angle
            }
            .withName("Wrist/${angle.getLoggingName()}")

    fun l1(): Command = setAngle(Angles.L1)
    fun l2(): Command = setAngle(Angles.L2)
    fun l3(): Command = setAngle(Angles.L3)
    fun l4(): Command = setAngle(Angles.L4)
    fun l2algae(): Command = setAngle(Angles.L2_ALGAE)
    fun l3algae(): Command = setAngle(Angles.L3_ALGAE)
    fun feeder(): Command = setAngle(Angles.FEEDER)
    fun retract(): Command = setAngle(Angles.ZERO)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
        Logger.recordOutput(
            "Pose",
            Pose3d(
                Translation3d(),
                Rotation3d(
                    Units.Degrees.zero(),
                    -io.inputs.angle,
                    Units.Degrees.zero()
                )
            )
        )
        ligament2d.setAngle(io.inputs.angle.`in`(Units.Degrees))
        Logger.recordOutput("Wrist/Mechanism2d", mechanism)
        Logger.recordOutput("Wrist/Setpoint", setpointValue)
    }
}
