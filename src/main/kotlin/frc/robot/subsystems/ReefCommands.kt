package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.elevator
import frc.robot.gripper
import frc.robot.wrist

private fun scoreCoral(endTrigger: Trigger): Command =
    sequence(
        waitUntil(endTrigger),
        gripper.outtake().until(gripper.hasCoral.negate()),
        moveDefaultPosition()
    )

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
