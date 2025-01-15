package frc.robot.subsystems.elevator

import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

class Elevator(private val io: ElevatorIO) : SubsystemBase() {

    fun setPosition(position: Distance): Command {
        Logger.recordOutput("Elevator/positionSetpoint",position )
        return run({ io.setHeight(position) })
    }

    fun setPower(percentOutput: DoubleSupplier): Command {
        return run({ io.setPower(percentOutput.asDouble) })
    }

    fun reset(): Command = runOnce(io::reset)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this.name, io.inputs)
    }
}
