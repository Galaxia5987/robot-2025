package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import frc.robot.extender
import frc.robot.roller
import java.util.function.BooleanSupplier

fun intakeAlgae(): Command = extender.extend().alongWith(roller.intake())

fun outtakeAlgae(retractTrigger: BooleanSupplier): Command =
    Commands.parallel(
        roller.outtake(),
        waitUntil(retractTrigger).andThen(retractIntake())
    )

fun farOuttakeAlgae(retractTrigger: BooleanSupplier): Command =
    Commands.parallel(
        roller.farOuttake(),
        waitUntil(retractTrigger).andThen(retractIntake())
    )

fun retractIntake(): Command = extender.retract()
