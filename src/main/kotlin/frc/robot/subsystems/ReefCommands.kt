package frc.robot.subsystems

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.*
import frc.robot.lib.getTranslation2d
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly

private val CORAL_OUTTAKE_TIMEOUT = Units.Seconds.of(0.15)

private fun shootCoral(): Command =
    runOnce({
        if (gripper.hasCoral.asBoolean) {
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    ReefscapeCoralOnFly(
                        driveSimulation!!.simulatedDriveTrainPose.translation,
                        getTranslation2d(x = 0.40, y = 0.0),
                        driveSimulation
                            .driveTrainSimulatedChassisSpeedsFieldRelative,
                        driveSimulation.simulatedDriveTrainPose.rotation,
                        elevator.height.invoke() + Units.Meters.of(0.50),
                        Units.MetersPerSecond.of(3.0),
                        wrist.angle.invoke() - Units.Degrees.of(35.0)
                    )
                )
        }
    })

private fun scoreCoral(endTrigger: Trigger): Command =
    sequence(
        waitUntil(endTrigger),
        gripper
            .outtake()
            .withTimeout(CORAL_OUTTAKE_TIMEOUT)
            .alongWith(
                either(shootCoral(), none()) { CURRENT_MODE == Mode.SIM }
            ),
        moveDefaultPosition()
    )

// TODO: Add Coral Simulation

private fun moveDefaultPosition(): Command =
    parallel(elevator.feeder(), wrist.feeder())

fun l1(outtakeTrigger: Trigger): Command =
    parallel(elevator.l1(), wrist.l1()).andThen(scoreCoral(outtakeTrigger))

fun l2(outtakeTrigger: Trigger): Command =
    parallel(elevator.l2(), wrist.l2()).andThen(scoreCoral(outtakeTrigger))

fun l3(outtakeTrigger: Trigger): Command =
    parallel(elevator.l3(), wrist.l3()).andThen(scoreCoral(outtakeTrigger))

fun l4(outtakeTrigger: Trigger): Command =
    parallel(elevator.l4(), wrist.l4()).andThen(scoreCoral(outtakeTrigger))

fun l3algae(): Command =
    parallel(elevator.l3Algae(), wrist.l3algae(), gripper.removeAlgae())

fun l2algae(): Command =
    parallel(elevator.l2Algae(), wrist.l2algae(), gripper.removeAlgae())

fun feeder(intakeTrigger: Trigger): Command =
    sequence(
        parallel(elevator.feeder(), wrist.feeder()),
        waitUntil(intakeTrigger),
        gripper.intake().until(gripper.hasCoral)
    )

fun retract(): Command = parallel(elevator.zero(), wrist.retract())
