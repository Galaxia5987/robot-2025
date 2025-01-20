package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Climber(private val io: ClimberIO) : SubsystemBase() {
    var inputs = io.inputs

    @AutoLogOutput
    private var isTouching = Trigger {
        inputs.sensorDistance < DISTANCE_THRESHOLD
    }

    @AutoLogOutput
    private var isLatchClosed = Trigger {
        inputs.latchPosition.isNear(CLOSE_LATCH_POSITION, LATCH_TOLERANCE)
    }

    @AutoLogOutput
    private var isStopperStuck = Trigger {
        inputs.stopperMotorCurrent.abs(Units.Amps) >= STOPPER_CURRENT_THRESHOLD.`in`(Units.Amps)
    }

    @AutoLogOutput
    private var isAttached = isLatchClosed.and(isTouching)

    @AutoLogOutput
    private val isFolded = Trigger {
        inputs.angle.isNear(FOLDED_ANGLE, FOLDED_TOLERANCE)
    }

    private fun setAngle(angle: Angle): Command = runOnce { io.setAngle(angle) }

    private fun setVoltage(voltage: Voltage): Command =
        Commands.startEnd(
            { io.setVoltage(voltage) },
            { io.setVoltage(Units.Volts.zero()) }
        )

    private fun setLatchPose(latchPose: Distance): Command = runOnce {
        io.setLatchPosition(latchPose)
    }

    fun closeLatch(): Command = setLatchPose(CLOSE_LATCH_POSITION)

    fun openLatch(): Command = setLatchPose(OPEN_LATCH_POSITION)

    fun lock(): Command = runOnce { io.closeStopper() }

    fun unlock(): Command =
        setVoltage(UNLOCK_VOLTAGE).withTimeout(0.15).andThen(io::openStopper)

    fun unfold() = setAngle(UNFOLDED_ANGLE)

    fun fold() = setAngle(FOLDED_ANGLE)

    fun climb(): Command = Commands.sequence(closeLatch(), fold(), lock())

    fun declimb(): Command = Commands.sequence(unlock(), unfold(), openLatch())

    override fun periodic() {
        io.updateInput()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}
