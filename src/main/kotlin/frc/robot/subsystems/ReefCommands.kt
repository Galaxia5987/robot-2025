package frc.robot.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.*
import frc.robot.lib.getRotation2d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.elevator.Positions
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly

private val CORAL_OUTTAKE_TIMEOUT = Units.Seconds.of(0.5)

private val CORAL_SHOOT_OFFSET =
    getTranslation2d(Units.Meters.of(0.40), Units.Meters.of(0.0))
private val GRIPPER_HEIGHT = Units.Meters.of(0.9)
private val CORAL_SHOOT_SPEED = Units.MetersPerSecond.of(3.0)
private val CORAL_L4_SHOOT_ANGLE = Units.Degrees.of(-85.0) //
private val WRIST_ANGLE_OFFSET = Units.Degrees.of(35.0)

private fun visualizeCoralOuttake(): Command =
    runOnce({
        if (!gripper.hasCoral.asBoolean) return@runOnce

        val arena = SimulatedArena.getInstance()
        val translation = driveSimulation!!.simulatedDriveTrainPose.translation
        val velocity =
            driveSimulation.driveTrainSimulatedChassisSpeedsFieldRelative
        val rotation = driveSimulation.simulatedDriveTrainPose.rotation
        val height = elevator.height.invoke() + GRIPPER_HEIGHT
        val angle =
            if (elevator.setpointName == Positions.L4) {
                CORAL_L4_SHOOT_ANGLE
            } else {
                wrist.angle.invoke() - WRIST_ANGLE_OFFSET
            }

        arena.addGamePieceProjectile(
            ReefscapeCoralOnFly(
                translation,
                CORAL_SHOOT_OFFSET,
                velocity,
                rotation,
                height,
                CORAL_SHOOT_SPEED,
                angle
            )
        )
    })

private fun scoreCoral(endTrigger: Trigger): Command =
    sequence(
        waitUntil(endTrigger),
        gripper
            .outtake()
            .withTimeout(CORAL_OUTTAKE_TIMEOUT)
            .alongWith(
                visualizeCoralOuttake().onlyIf { CURRENT_MODE != Mode.REAL }
            ),
        moveDefaultPosition()
    )

private fun scoreCoralL4(endTrigger: Trigger): Command =
    sequence(
        waitUntil(endTrigger),
        gripper
            .fastOuttake()
            .withTimeout(0.8)
            .alongWith(
                visualizeCoralOuttake().onlyIf { CURRENT_MODE != Mode.REAL }
            ),
        wrist.retract(),
        waitSeconds(0.4),
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
    parallel(elevator.l4(), wrist.l4()).andThen(scoreCoralL4(outtakeTrigger))

fun l3algae(retractTrigger: Trigger): Command =
    parallel(elevator.l3Algae(), wrist.l3algae(), gripper.removeAlgae())
        .until(retractTrigger)
        .andThen(moveDefaultPosition())

fun l2algae(retractTrigger: Trigger): Command =
    parallel(elevator.l2Algae(), wrist.l2algae(), gripper.removeAlgae())
        .until(retractTrigger)
        .andThen(moveDefaultPosition())

fun feeder(intakeTrigger: Trigger): Command =
    sequence(
        parallel(elevator.feeder(), wrist.feeder()),
        waitUntil(intakeTrigger),
        gripper.intake().until(gripper.hasCoral).andThen(moveDefaultPosition())
    )

fun retract(): Command = parallel(elevator.zero(), wrist.retract())
