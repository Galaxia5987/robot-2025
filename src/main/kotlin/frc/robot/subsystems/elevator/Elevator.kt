package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.DoubleSupplier
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Elevator(private val io: ElevatorIO) : SubsystemBase() {

    @AutoLogOutput var positionSetpoint: Distance = Units.Millimeters.of(0.0)

    fun setPosition(position: Distance): Command = runOnce{
        positionSetpoint = position
        io.setHeight(position)
    }

    fun setPower(percentOutput: DoubleSupplier): Command = run { io.setPower(percentOutput.asDouble)}

    fun reset(): Command = runOnce(io::resetAbsoluteEncoder)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this.name, io.inputs)
    }
}
