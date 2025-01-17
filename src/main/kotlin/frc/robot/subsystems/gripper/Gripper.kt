package frc.robot.subsystems.gripper

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import org.littletonrobotics.junction.Logger

class Gripper private constructor(private val io: GripperIO) : SubsystemBase() {

    companion object {
        @Volatile private var instance: Gripper? = null

        fun initialize(io: GripperIO) {
            synchronized(this) {
                if (Gripper.instance == null) {
                    instance = Gripper(io)
                }
            }
        }
        fun getInstance(): Gripper {
            return Gripper.instance
                ?: throw IllegalStateException(
                    "Gripper has not been initialized. Call initialize(io: GripperIO) first."
                )
        }
    }

    fun setPosition(position: Angle): Command {
        return run({ io.setRotate(position) })
    }

    fun setPower(percentOutput: DoubleSupplier): Command {
        return run({ io.setPower(percentOutput.asDouble) })
    }

    fun reset(): Command = Commands.runOnce(io::reset)

    override fun periodic() {
        Logger.recordOutput("CheckGripper", false)

        io.updateInputs()
        Logger.processInputs(this.name, io.inputs)
        val Rotates = getRotateSIM()
        Logger.recordOutput(
            "GripperPose3D",
            Pose3d(
                0.13567083,
                0.0,
                0.8 +
                    RobotContainer.elevator.io.inputs.height
                        .baseUnitMagnitude(),
                Rotation3d(0.0, Rotates[0], 0.0)
            )
        )
    }

    private fun getRotateSIM(): Array<Double> {
        var delta = 0.007
        val currentSetRotate: Double = io.inputs.Setpoint.baseUnitMagnitude()
        var GripperRotate: Double = 0.0
        var currentRotate = io.inputs.Rotate.baseUnitMagnitude()
        if (
            currentSetRotate < currentRotate &&
                abs(currentSetRotate - currentRotate) > delta
        ) {
            GripperRotate = currentRotate - delta
            io.inputs.Rotate =
                io.inputs.Rotate.minus(
                    edu.wpi.first.units.Units.Rotations.of(delta)
                )
        } else {
            if (
                currentSetRotate > currentRotate &&
                    abs(currentSetRotate - currentRotate) > delta
            ) {
                GripperRotate = currentRotate + delta
                io.inputs.Rotate =
                    io.inputs.Rotate.plus(
                        edu.wpi.first.units.Units.Rotations.of(delta)
                    )
                //                carrigeExtend =(io.inputs.Extend.baseUnitMagnitude()+0.04)
                //                io.inputs.Rotate =
                // edu.wpi.first.units.Units.Rotations.of(currentSetExtend)
            } else {
                GripperRotate = (io.inputs.Rotate.baseUnitMagnitude())
            }
        }

        return arrayOf(GripperRotate)
    }
}
