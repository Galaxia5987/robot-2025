package frc.robot.subsystems.intake

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

class Intake private constructor(private val io: IntakeIO) : SubsystemBase() {

    companion object {
        @Volatile private var instance: Intake? = null

        fun initialize(io: IntakeIO) {
            synchronized(this) {
                if (Intake.instance == null) {
                    instance = Intake(io)
                }
            }
        }
        fun getInstance(): Intake {
            return Intake.instance
                ?: throw IllegalStateException(
                    "Intake has not been initialized. Call initialize(io: IntakeIO) first."
                )
        }
    }

    fun setPosition(position: Distance): Command {
        return run({ io.setExtend(position) })
    }

    fun setPower(percentOutput: DoubleSupplier): Command {
        return run({ io.setPower(percentOutput.asDouble) })
    }

    fun reset(): Command = Commands.runOnce(io::reset)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this.name, io.inputs)
        val Extends = getExtendSIM()
        Logger.recordOutput(
            "IntakePose3D-1",
            Pose3d(0.0 + Extends[0], 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0))
        )
    }

    private fun getExtendSIM(): Array<Double> {
        var delta = 0.007
        val currentSetExtend: Double = io.inputs.Setpoint.baseUnitMagnitude()
        var IntkaeExtend: Double = 0.0
        var currentExtend = io.inputs.Extend.baseUnitMagnitude()
        if (
            currentSetExtend < currentExtend &&
                abs(currentSetExtend - currentExtend) > delta
        ) {
            IntkaeExtend = (io.inputs.Extend.baseUnitMagnitude() - delta)
            Logger.recordOutput("IntakeExtend", IntkaeExtend)
            io.inputs.Extend = Units.Meters.of((IntkaeExtend))
        } else {
            if (
                currentSetExtend > currentExtend &&
                    abs(currentSetExtend - currentExtend) > delta
            ) {
                IntkaeExtend = (io.inputs.Extend.baseUnitMagnitude() + delta)
                //                carrigeExtend =(io.inputs.Extend.baseUnitMagnitude()+0.04)
                io.inputs.Extend = Units.Meters.of((IntkaeExtend))
            } else {
                IntkaeExtend = (io.inputs.Extend.baseUnitMagnitude())
            }
        }

        return arrayOf(IntkaeExtend)
    }
}
