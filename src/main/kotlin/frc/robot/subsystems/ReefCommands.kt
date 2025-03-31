package frc.robot.subsystems

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.CURRENT_MODE
import frc.robot.Mode
import frc.robot.autonomous.isL4
import frc.robot.autonomous.shouldOpenElevator
import frc.robot.driveSimulation
import frc.robot.elevator
import frc.robot.extender
import frc.robot.gripper
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.elevator.Positions
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.intake.intakeToGripper
import frc.robot.subsystems.wrist.MANUAL_CONTROL_VOLTAGE
import frc.robot.swerveDrive
import frc.robot.wrist
import java.util.function.BooleanSupplier
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

fun outtakeCoralL3Alignment(): Command =
    sequence(
        (gripper
                .outtakeL3()
                .withTimeout(0.3)
                .andThen(
                    gripper
                        .outtakeL3()
                        .until(gripper.hasCoral.negate())
                        .alongWith(
                            visualizeCoralOuttake().onlyIf {
                                CURRENT_MODE != Mode.REAL
                            }
                        )
                ))
            .withTimeout(0.5),
        gripper.slowOuttake(true).withTimeout(0.15),
        wrist.skyward().alongWith(elevator.zero())
    )

fun outtakeCoralAlignment(isReverse: Boolean = false): Command =
    sequence(
        (gripper
                .outtake(isReverse)
                .withTimeout(0.3)
                .andThen(
                    gripper
                        .outtake(isReverse)
                        .until(gripper.hasCoral.negate())
                        .alongWith(
                            visualizeCoralOuttake().onlyIf {
                                CURRENT_MODE != Mode.REAL
                            }
                        )
                ))
            .withTimeout(0.5),
        gripper.slowOuttake(!isReverse).withTimeout(0.15),
        wrist.skyward().alongWith(elevator.zero())
    )

fun outtakeCoralManual(isReverse: Boolean = false): Command =
    sequence(
        (gripper
                .outtake(isReverse)
                .withTimeout(0.3)
                .andThen(
                    gripper
                        .outtake(isReverse)
                        .until(gripper.hasCoral.negate())
                        .alongWith(
                            visualizeCoralOuttake().onlyIf {
                                CURRENT_MODE != Mode.REAL
                            }
                        )
                ))
            .withTimeout(0.5),
        gripper.slowOuttake(!isReverse).withTimeout(0.15),
        wrist.skyward(),
        WaitCommand(0.2),
        elevator.zero()
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
        gripper.slowOuttake(true).withTimeout(0.15),
        moveDefaultPosition(false)
            .onlyIf(
                gripper.hasCoral.negate().and { !DriverStation.isAutonomous() }
            )
    )

fun outtakeL1(): Command =
    sequence(
        (gripper
            .slowOuttake(true)
            .until(gripper.hasCoral.negate())
            .alongWith(
                visualizeCoralOuttake().onlyIf { CURRENT_MODE != Mode.REAL }
            )),
        moveDefaultPosition(false)
            .onlyIf(
                gripper.hasCoral.negate().and { !DriverStation.isAutonomous() }
            )
    )

fun outtakeL2(): Command =
    sequence(
        (gripper
                .outtake(true)
                .withTimeout(0.3)
                .andThen(
                    gripper
                        .outtake(true)
                        .until(gripper.hasCoral.negate())
                        .alongWith(
                            visualizeCoralOuttake().onlyIf {
                                CURRENT_MODE != Mode.REAL
                            }
                        )
                ))
            .withTimeout(0.5),
        gripper.slowOuttake(false).withTimeout(0.15),
        moveDefaultPosition(false, { false }),
        WaitCommand(0.2),
        swerveDrive
            .run {
                swerveDrive.limitlessRunVelocity(ChassisSpeeds(-0.8, 0.0, 0.0))
            }
            .withTimeout(0.3),
        swerveDrive.run { swerveDrive.stop() }.withTimeout(0.25)
    )

// TODO: Add Coral Simulation

fun moveDefaultPosition(
    moveWristUp: Boolean,
    zeroWrist: BooleanSupplier = BooleanSupplier { true }
): Command =
    Commands.defer(
            {
                sequence(
                    wrist.max().onlyIf { moveWristUp },
                    elevator.feeder(),
                    waitUntil(wrist.atSetpoint),
                    if (zeroWrist.asBoolean) wrist.retract() else wrist.feeder()
                )
            },
            setOf(elevator, wrist)
        )
        .withName("Reef/Move default position")

fun l1(): Command =
    parallel(elevator.l1(), wrist.l1(), runOnce({ isL4 = Trigger { false } }))
        .withName("Reef/Move L1")

fun l2(): Command =
    parallel(elevator.l2(), wrist.l2(), runOnce({ isL4 = Trigger { false } }))
        .withName("Reef/Move L2")

fun l3(): Command =
    parallel(elevator.l3(), wrist.l3(), runOnce({ isL4 = Trigger { false } }))
        .withName("Reef/Move L3")

fun l3Manual(): Command =
    parallel(elevator.zero(), wrist.l3Manual()).withName("Reef/Move L3 Manual")

fun l4(): Command =
    parallel(elevator.l4(), wrist.l4(), runOnce({ isL4 = Trigger { true } }))
        .withName("Reef/Move L4")

fun alignL2(): Command =
    parallel(
            elevator.alignL2(),
            wrist.alignL2(),
            runOnce({ isL4 = Trigger { true } })
        )
        .withName("Reef/Auto L2")

fun alignmentSetpointL4(): Command =
    parallel(
            elevator.alignL4(),
            WaitCommand(0.18).andThen(wrist.alignL4()),
            runOnce({ isL4 = Trigger { true } })
        )
        .withName("Reef/Auto L4")

fun raiseElevatorAtDistance(elevatorCommand: Command): Command =
    sequence(
            waitUntil(shouldOpenElevator),
            elevatorCommand,
            waitUntil(elevator.almostAtSetpoint.and(wrist.almostAtSetpoint))
        )
        .withName("Reef/raiseElevatorAtDistance")

fun l2algae(retractTrigger: Trigger): Command =
    parallel(elevator.l2Algae(), wrist.l2algae(), gripper.removeAlgae())
        .until(retractTrigger)
        .andThen(moveDefaultPosition(false))
        .withName("Reef/L2 Algae")

fun l2algaePickup(): Command =
    sequence(
        elevator.l2AlgaePickup(),
        wrist.l2algaePickup(),
        gripper.intakeAlgae(),
        waitUntil(gripper.hasAlgae),
        wrist.l2algaePickupEnd(),
    )

fun l3algaePickup(): Command =
    sequence(
        elevator.l3AlgaePickup(),
        wrist.l3algaePickup(),
        gripper.intakeAlgae(),
        waitUntil(gripper.hasAlgae),
        wrist.l3algaePickupEnd()
    )

fun l3algae(retractTrigger: Trigger): Command =
    parallel(elevator.l3Algae(), wrist.l3algae(), gripper.removeAlgae())
        .until(retractTrigger)
        .andThen(moveDefaultPosition(false))
        .withName("Reef/L3 Algae")

fun feeder(
    intakeTrigger: Trigger,
    zeroWrist: BooleanSupplier = BooleanSupplier { true }
): Command =
    sequence(
            parallel(elevator.feeder(), wrist.feeder()),
            waitUntil(intakeTrigger),
            gripper
                .intake()
                .until(gripper.hasCoral)
                .andThen(moveDefaultPosition(false, zeroWrist))
        )
        .withName("Reef/Feeder")

fun autonomousFeeder(): Command =
    sequence(
            parallel(elevator.feeder(), wrist.feeder()),
            gripper
                .intake()
                .until(gripper.autoHasCoral)
                .andThen(moveDefaultPosition(false, { false }))
        )
        .withName("Reef/Feeder")

fun blockedFeeder(
    intakeTrigger: Trigger,
    zeroWrist: BooleanSupplier = BooleanSupplier { true }
): Command =
    sequence(
            parallel(elevator.blockedFeeder(), wrist.blockedFeeder()),
            waitUntil(intakeTrigger),
            gripper
                .intake()
                .until(gripper.hasCoral)
                .andThen(moveDefaultPosition(false, zeroWrist))
        )
        .withName("Reef/Blocked Feeder")

fun retract(): Command = parallel(elevator.zero(), wrist.retract())

fun intakeAlgaeToGripper(gripperTrigger: Trigger): Command =
    parallel(
            wrist.floorAlgae(),
            elevator.zero(),
            intakeAlgae(),
            gripper.intakeAlgae()
        )
        .until(gripperTrigger)
        .andThen(intakeToGripper().withTimeout(0.8))
        .andThen(
            wrist.setVoltage(MANUAL_CONTROL_VOLTAGE).withTimeout(1.0),
            wrist.max().until(wrist.atSetpoint),
            extender.retract()
        )

fun netAlgae(outtakeTrigger: Trigger): Command =
    sequence(
        elevator.zero(),
        waitUntil(outtakeTrigger),
        elevator.net(),
        WaitUntilCommand {
            elevator.height.invoke().isNear(Units.Meters.of(0.75 + 0.5), 0.5)
        }, // anything above 0.75
        wrist.l4(),
        WaitUntilCommand {
            wrist.angle
                .invoke()
                .isNear(
                    Units.Degrees.of(210.0 - 10),
                    10.0
                ) // when lower than 153
        },
        gripper.outtakeAlgae().withTimeout(0.2),
        moveDefaultPosition(true)
    )
