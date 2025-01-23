package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.extender
import frc.robot.roller

fun algaeIntake(): Command = extender.extend().alongWith(roller.intake())

fun algaeOuttake(endTrigger: Trigger): Command =
    Commands.parallel(
        roller.outtake(),
        Commands.waitUntil(endTrigger).andThen(retractIntake())
    )

fun algaeFarOuttake(endTrigger: Trigger): Command =
    Commands.parallel(
        roller.farOuttake(),
        Commands.waitUntil(endTrigger).andThen(retractIntake())
    )

fun retractIntake(): Command = extender.retract()
