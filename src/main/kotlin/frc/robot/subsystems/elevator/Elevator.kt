package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.DoubleSupplier
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Elevator(private val io: ElevatorIO) : SubsystemBase() {
    private val mechanism = LoggedMechanism2d(3.0, 3.0)
    private val root = mechanism.getRoot("Elevator", 2.0, 0.0)
    private val elevatorLigament = root.append(LoggedMechanismLigament2d("ElevatorLigament", 1.0, 90.0))
    @AutoLogOutput private var positionSetpoint: Distance = Units.Millimeters.of(0.0)

    fun setPosition(position: Distance): Command = runOnce {
        positionSetpoint = position
        io.setHeight(position)
    }

    fun setPower(percentOutput: DoubleSupplier): Command = run {
        io.setPower(percentOutput.asDouble)
    }

    fun resetAbsoluteEncoder(): Command = runOnce(io::resetAbsoluteEncoder)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this.name, io.inputs)
        Logger.recordOutput("Elevator/Mechanism2d", mechanism)

        elevatorLigament.length = io.inputs.height.`in`(Units.Meters)
    }
}
