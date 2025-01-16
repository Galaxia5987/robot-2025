package frc.robot.subsystems.elevator

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.DoubleSupplier
import kotlin.math.abs
import org.littletonrobotics.junction.Logger

class Elevator private constructor(val io: ElevatorIO) : SubsystemBase() {

    companion object {
        @Volatile private var instance: Elevator? = null

        fun initialize(io: ElevatorIO) {
            synchronized(this) {
                if (Elevator.instance == null) {
                    instance = Elevator(io)
                }
            }
        }
        fun getInstance(): Elevator {
            return Elevator.instance
                ?: throw IllegalStateException(
                    "Elevator has not been initialized. Call initialize(io: ElevatorIO) first."
                )
        }
    }

    fun setPosition(position: Distance): Command {
        return run({ io.setHeight(position) })
    }

    fun setPower(percentOutput: DoubleSupplier): Command {
        return run({ io.setPower(percentOutput.asDouble) })
    }

    fun reset(): Command = Commands.runOnce(io::resetAbsoluteEncoder)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this.name, io.inputs)
        // io.inputs.Setpoint=io.inputs.Setpoint.plus(Units.Meters.of(0.001))
        val heights = getHeightSIM()
        Logger.recordOutput(
            "ElevatorPose3D-2",
            Pose3d(0.0, 0.0, 0.0 + heights[1], Rotation3d(0.0, 0.0, 0.0))
        )
        //
        // Logger.recordOutput("ElevatorPose3D-1",Pose3d(0.13567083,0.0,0.12+(io.inputs.heightSetpoint.baseUnitMagnitude()), Rotation3d(0.0,0.0,0.0)))
        Logger.recordOutput(
            "ElevatorPose3D-1",
            Pose3d(0.0, 0.0, 0.0 + heights[0], Rotation3d(0.0, 0.0, 0.0))
        )
    }

    private fun getHeightSIM(): Array<Double> {
        val currentSetHeight: Double = io.inputs.setpoint.baseUnitMagnitude()
        var delta = 0.007
        var Elv1Height: Double = 0.0
        var carrigeHeight: Double = 0.0
        var currentHight = io.inputs.height.baseUnitMagnitude()
        if (
            currentSetHeight < currentHight &&
                abs(currentHight - currentSetHeight) > delta
        ) {
            Elv1Height = (io.inputs.height.baseUnitMagnitude() - delta) / 2
            carrigeHeight = (io.inputs.height.baseUnitMagnitude() - delta)
            Logger.recordOutput("Elv1height", Elv1Height)
            Logger.recordOutput("carrigeHeight", carrigeHeight)
            io.inputs.height = Units.Meters.of((carrigeHeight))
        } else {
            if (
                currentSetHeight > currentHight &&
                    abs(currentHight - currentSetHeight) > delta
            ) {
                Elv1Height = (io.inputs.height.baseUnitMagnitude() + delta) / 2
                carrigeHeight = (io.inputs.height.baseUnitMagnitude() + delta)
                io.inputs.height = Units.Meters.of((carrigeHeight))
            } else {
                Elv1Height = (io.inputs.height.baseUnitMagnitude()) / 2
                carrigeHeight = (io.inputs.height.baseUnitMagnitude())
            }
        }

        return arrayOf(Elv1Height, carrigeHeight)
    }
}
