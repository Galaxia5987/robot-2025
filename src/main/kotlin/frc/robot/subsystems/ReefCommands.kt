package frc.robot.subsystems

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.CURRENT_MODE
import frc.robot.Mode
import frc.robot.autonomous.isL4
import frc.robot.autonomous.shouldOpenElevator
import frc.robot.driveSimulation
import frc.robot.elevator
import frc.robot.gripper
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.elevator.Positions
import frc.robot.swerveDrive
import frc.robot.wrist
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly

private val CORAL_OUTTAKE_TIMEOUT = Units.Seconds.of(0.5)

private val CORAL_SHOOT_OFFSET =
    getTranslation2d(Units.Meters.of(0.40), Units.Meters.of(0.0))
private val GRIPPER_HEIGHT = Units.Meters.of(0.50)
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

fun outtakeCoralAndDriveBack(): Command =
    sequence(
        (gripper
                .outtake()
                .until(gripper.hasCoral.negate())
                .alongWith(
                    visualizeCoralOuttake().onlyIf { CURRENT_MODE != Mode.REAL }
                ))
            .withTimeout(0.5),
        gripper.slowOuttake(true).withTimeout(0.1),
        swerveDrive
            .run { swerveDrive.runVelocity(ChassisSpeeds(-0.5, 0.0, 0.0)) }
            .withTimeout(0.1)
            .alongWith(wrist.max()),
        moveDefaultPosition().onlyIf(gripper.hasCoral.negate())
    )

fun outtakeCoral(): Command =
    sequence(
        (gripper
                .outtake()
                .until(gripper.hasCoral.negate())
                .alongWith(
                    visualizeCoralOuttake().onlyIf { CURRENT_MODE != Mode.REAL }
                ))
            .withTimeout(0.5),
        gripper.slowOuttake(true).withTimeout(0.1),
        moveDefaultPosition()
            .onlyIf(
                gripper.hasCoral.negate().and { !DriverStation.isAutonomous() }
            )
    )

// TODO: Add Coral Simulation

fun moveDefaultPosition(): Command =
    sequence(wrist.max(), elevator.feeder(), waitUntil(elevator.atSetpoint), wrist.l1())
        .withName("Reef/Move default position")

fun l1(): Command = parallel(elevator.l1(), wrist.l1(), runOnce({isL4 = Trigger{false}})).withName("Reef/Move L1")

fun l2(): Command = parallel(elevator.l2(), wrist.l2(), runOnce({isL4 = Trigger{false}})).withName("Reef/Move L2")

fun l3(): Command = parallel(elevator.l3(), wrist.l3(), runOnce({isL4 = Trigger{false}})).withName("Reef/Move L3")

fun l4(): Command = parallel(elevator.l4(), wrist.l4(), runOnce({isL4 = Trigger{true}})).withName("Reef/Move L4")

fun alignL4(): Command =
    parallel(elevator.alignL4(), wrist.alignL4(), runOnce({isL4 = Trigger{true}})).withName("Reef/Auto L4")

fun raiseElevatorAtDistance(elevatorCommand: Command): Command =
    waitUntil(shouldOpenElevator)
        .andThen(
            elevatorCommand.until(elevator.atSetpoint.and(wrist.atSetpoint))
        )
        .withName("Reef/raiseElevatorAtDistance")

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
            parallel(elevator.blockedFeeder(), wrist.blockedFeeder()),
            waitUntil(intakeTrigger),
            gripper
                .intake()
                .until(gripper.hasCoral)
                .andThen(moveDefaultPosition())
        )
        .withName("Reef/Blocked Feeder")

fun retract(): Command = parallel(elevator.zero(), wrist.retract())
