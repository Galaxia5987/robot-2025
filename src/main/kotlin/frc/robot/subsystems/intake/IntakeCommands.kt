package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.sequence
import frc.robot.extender
import frc.robot.roller
import java.util.function.BooleanSupplier

fun intakeAlgae(): Command = extender.extend().alongWith(roller.intake())

fun outtakeAlgae(retractTrigger: BooleanSupplier): Command =
    sequence(roller.outtake().until(retractTrigger), retractIntake())

fun farOuttakeAlgae(retractTrigger: BooleanSupplier): Command =
    sequence(
        roller.farOuttake().until(retractTrigger),
        retractIntake()
    )

fun retractIntake(): Command = extender.retract()
