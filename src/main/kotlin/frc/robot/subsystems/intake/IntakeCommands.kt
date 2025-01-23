package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import frc.robot.extender
import frc.robot.roller
import java.util.function.BooleanSupplier

fun algaeIntake(): Command = extender.extend().alongWith(roller.intake())

fun algaeOuttake(endTrigger: BooleanSupplier): Command =
    Commands.parallel(
        roller.outtake(),
        waitUntil(endTrigger).andThen(retractIntake())
    )

fun algaeFarOuttake(endTrigger: BooleanSupplier): Command =
    Commands.parallel(
        roller.farOuttake(),
        waitUntil(endTrigger).andThen(retractIntake())
    )

fun retractIntake(): Command = extender.retract()
