package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.*
import frc.robot.autonomous.ALIGNMENT_FORWARD_VELOCITY
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.drive.TunerConstants
import frc.robot.subsystems.elevator.Positions
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly

private val CORAL_OUTTAKE_TIMEOUT = Units.Seconds.of(0.5)

private val CORAL_SHOOT_OFFSET =
    getTranslation2d(Units.Meters.of(0.40), Units.Meters.of(0.0))
private val GRIPPER_HEIGHT = Units.Meters.of(0.9)
private val CORAL_SHOOT_SPEED = Units.MetersPerSecond.of(3.0)
private val CORAL_L4_SHOOT_ANGLE = Units.Degrees.of(-80.0)
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

fun visualizeCoralOuttakeIfNeeded(): Command =
    visualizeCoralOuttake().onlyIf { CURRENT_MODE != Mode.REAL }

private fun scoreCoral(endTrigger: Trigger): Command =
    sequence(
        waitUntil(endTrigger),
        gripper
            .outtake()
            .withTimeout(CORAL_OUTTAKE_TIMEOUT)
            .alongWith(visualizeCoralOuttakeIfNeeded()),
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
        moveDefaultPosition()
    )

// TODO: Add Coral Simulation

fun moveDefaultPosition(): Command =
    sequence(elevator.feeder(), waitUntil(elevator.atSetpoint), wrist.feeder())
        .withName("Reef/Move default position")

fun l1(outtakeTrigger: Trigger): Command =
    l1().andThen(scoreCoral(outtakeTrigger)).withName("Reef/L1")

fun l1(): Command = parallel(elevator.l1(), wrist.l1()).withName("Reef/Move L1")

fun l2(outtakeTrigger: Trigger): Command =
    l2().andThen(scoreCoral(outtakeTrigger)).withName("Reef/L2")

fun l2(): Command = parallel(elevator.l2(), wrist.l2()).withName("Reef/Move L2")

fun l3(outtakeTrigger: Trigger): Command =
    l3().andThen(scoreCoral(outtakeTrigger)).withName("Reef/L3")

fun l3(): Command = parallel(elevator.l3(), wrist.l3()).withName("Reef/Move L3")

fun l4(outtakeTrigger: Trigger): Command =
    l4().andThen(scoreCoralL4(outtakeTrigger)).withName("Reef/L4")

fun dumbL4(outtakeTrigger: Trigger): Command =
    l4().andThen(waitUntil(outtakeTrigger),
        gripper
            .fastOuttake()
            .withTimeout(0.8)
            .alongWith(
                visualizeCoralOuttake().onlyIf { CURRENT_MODE != Mode.REAL }
            )).withName("Reef/DumbL4").andThen(
                run({
                    swerveDrive.runVelocity(
                        ChassisSpeeds(
                            -1.0,
                            0.0,
                            0.0
                        )
                    )}
                ).withTimeout(0.4).andThen(moveDefaultPosition())
            )

fun l4(): Command = parallel(elevator.l4(), wrist.l4()).withName("Reef/Move L4")

fun pathfindFeeder(outtakeTrigger: Trigger): Command =
    AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("Feeder"),
            TunerConstants.PATH_CONSTRAINTS
        )
        .andThen(feeder(outtakeTrigger))

fun l2algae(retractTrigger: Trigger): Command =
    parallel(elevator.l2Algae(), wrist.l2algae(), gripper.removeAlgae())
        .until(retractTrigger)
        .andThen(moveDefaultPosition())
        .withName("Reef/L2 Algae")

fun l3algae(retractTrigger: Trigger): Command =
    parallel(elevator.l3Algae(), wrist.l3algae(), gripper.removeAlgae())
        .until(retractTrigger)
        .andThen(moveDefaultPosition())
        .withName("Reef/L3 Algae")

fun feeder(intakeTrigger: Trigger): Command =
    sequence(
            parallel(elevator.feeder(), wrist.feeder()),
            waitUntil(intakeTrigger),
            gripper
                .intake()
                .until(gripper.hasCoral)
                .andThen(moveDefaultPosition())
        )
        .withName("Reef/Feeder")

fun blockedFeeder(intakeTrigger: Trigger): Command =
    sequence(
            parallel(elevator.feeder(), wrist.blockedFeeder()),
            waitUntil(intakeTrigger),
            gripper
                .intake()
                .until(gripper.hasCoral)
                .andThen(moveDefaultPosition())
        )
        .withName("Reef/Blocked Feeder")

fun retract(): Command = parallel(elevator.zero(), wrist.retract())
