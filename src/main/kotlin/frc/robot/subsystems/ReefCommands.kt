package frc.robot.subsystems

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.wrist.Wrist

private val CORAL_OUTTAKE_TIMEOUT = Units.Seconds.of(0.15)

class ReefCommands(
    private val elevator: Elevator,
    private val gripper: Gripper,
    private val wrist: Wrist,
) {

    private fun scoreCoral(endTrigger: Trigger): Command =
        sequence(
            waitUntil(endTrigger),
            gripper.outtake().withTimeout(CORAL_OUTTAKE_TIMEOUT),
            moveDefaultPosition()
        )

    private fun moveDefaultPosition(): Command =
        parallel(elevator.feeder(), wrist.feeder())

    fun moveL1(endTrigger: Trigger): Command =
        parallel(elevator.l1(), wrist.l1()).andThen(scoreCoral(endTrigger))

    fun moveL2(endTrigger: Trigger): Command =
        parallel(elevator.l2(), wrist.l2()).andThen(scoreCoral(endTrigger))

    fun moveL3(endTrigger: Trigger): Command =
        parallel(elevator.l3(), wrist.l3()).andThen(scoreCoral(endTrigger))

    fun moveL4(endTrigger: Trigger): Command =
        parallel(elevator.l4(), wrist.l4()).andThen(scoreCoral(endTrigger))

    fun moveL3algae(): Command =
        parallel(elevator.l3Algae(), wrist.l3algae(), gripper.removeAlgae())

    fun moveL2algae(): Command =
        parallel(elevator.l2Algae(), wrist.l2algae(), gripper.removeAlgae())

    fun moveFeeder(endTrigger: Trigger): Command =
        sequence(
            parallel(elevator.feeder(), wrist.feeder()),
            waitUntil(endTrigger),
            gripper.intake().until(gripper.hasCoral)
        )

    fun retract(): Command = parallel(elevator.zero(), wrist.retract())
}
