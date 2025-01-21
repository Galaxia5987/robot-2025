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
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Climber(private val io: ClimberIO) : SubsystemBase() {
    var inputs = io.inputs

    @AutoLogOutput
    private val isTouching =
        Trigger { inputs.sensorDistance < DISTANCE_THRESHOLD }.debounce(1.0)

    @AutoLogOutput
    private val isLatchClosed = Trigger {
        inputs.latchPosition.isNear(CLOSE_LATCH_POSITION, LATCH_TOLERANCE)
    }

    @AutoLogOutput
    private val isStopperStuck = Trigger {
        inputs.stopperMotorCurrent.abs(Units.Amps) >=
            STOPPER_CURRENT_THRESHOLD.`in`(Units.Amps)
    }

    @AutoLogOutput private val isAttached = isLatchClosed.and(isTouching)

    @AutoLogOutput
    private val isFolded = Trigger {
        inputs.angle.isNear(FOLDED_ANGLE, FOLDED_TOLERANCE)
    }

    @AutoLogOutput
    private val isUnfolded = Trigger {
        inputs.angle.isNear(UNFOLDED_ANGLE, FOLDED_TOLERANCE)
    }

    @AutoLogOutput
    private val atSetpoint = Trigger {
        inputs.angle.isNear(setpoint, FOLDED_TOLERANCE)
    }

    @AutoLogOutput private var setpoint = Units.Rotations.zero()

    @AutoLogOutput private var mechanism = LoggedMechanism2d(3.0, 2.0)
    private var root = mechanism.getRoot("Climber", 1.0, 1.0)
    private val ligament =
        root.append(LoggedMechanismLigament2d("ClimberLigament", 0.27003, 90.0))

    private fun setAngle(angle: Angle): Command = runOnce {
        io.setAngle(angle)
        setpoint = angle
    }

    private fun setVoltage(voltage: Voltage): Command =
        startEnd(
            { io.setVoltage(voltage) },
            { io.setVoltage(Units.Volts.zero()) }
        )

    private fun setLatchPose(latchPose: Distance): Command = runOnce {
        io.setLatchPosition(latchPose)
    }

    private fun setStopperPower(power: Double): Command = runOnce {
        io.setStopperPower(power)
    }

    fun closeLatch(): Command = setLatchPose(CLOSE_LATCH_POSITION)

    fun openLatch(): Command = setLatchPose(OPEN_LATCH_POSITION)

    fun lock(): Command =
        Commands.sequence(
            setStopperPower(LOCK_POWER),
            Commands.waitUntil(isStopperStuck),
            setStopperPower(0.0)
        )

    fun unlock(): Command =
        Commands.sequence(
            setStopperPower(UNLOCK_POWER),
            Commands.waitUntil(isStopperStuck),
            setStopperPower(0.0)
        )

    fun unfold() = setAngle(UNFOLDED_ANGLE)

    fun fold() = setAngle(FOLDED_ANGLE)

    fun climb(): Command =
        Commands.sequence(
            closeLatch(),
            Commands.waitUntil(isLatchClosed),
            fold(),
            Commands.waitUntil(isFolded),
            lock()
        )

    fun declimb(): Command =
        Commands.sequence(
            unlock(),
            unfold(),
            Commands.waitUntil(isUnfolded),
            openLatch()
        )

    override fun periodic() {
        io.updateInput()
        Logger.processInputs(this::class.simpleName, io.inputs)

        ligament.angle = inputs.angle.`in`(Units.Degrees)
        Logger.recordOutput("Climber/Mechanism2d", mechanism)
        Logger.recordOutput("Climber/setpoint", setpoint)
    }
}
