package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.extender
import frc.robot.roller

fun intake(): Command = extender.extend().alongWith(roller.intake())

fun outtake(endTrigger: Trigger): Command =
    Commands.parallel(roller.outtake(), Commands.waitUntil(endTrigger).andThen(retract()))

fun farOuttake(endTrigger: Trigger): Command =
    Commands.parallel(roller.farOuttake(), Commands.waitUntil(endTrigger).andThen(retract()))

fun retract(): Command = extender.retract()
