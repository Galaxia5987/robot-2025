package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.wrist.Wrist

class ReefCommands(private val elevator: Elevator, private val gripper: Gripper, private val wrist: Wrist) {
    fun releaseCoral(): Command = gripper.outtake()

    // `.onFalse` will be `releaseCoral`
    fun moveL1(): Command = Commands.parallel(elevator.l1(), wrist.l1())
    fun moveL2(): Command = Commands.parallel(elevator.l2(), wrist.l2())
    fun moveL3(): Command = Commands.parallel(elevator.l3(), wrist.l3())
    fun moveL4(): Command = Commands.parallel(elevator.l4(), wrist.l4())

    fun moveL2algae(): Command = Commands.parallel(elevator.l2Algae(), wrist.l2algae(), gripper.removeAlgae())
    fun moveL3algae(): Command = Commands.parallel(elevator.l3Algae(), wrist.l3algae(), gripper.removeAlgae())

    fun intakeCoral(): Command = gripper.intake()

    // `.onFalse` will be `intakeCoral`
    fun moveFeeder(): Command = Commands.parallel(elevator.feeder(), wrist.feeder())

}