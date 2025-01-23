package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.extender
import frc.robot.roller
import java.util.function.BooleanSupplier

fun intakeAlgae(): Command = extender.extend().alongWith(roller.intake())

fun outtakeAlgae(retractTrigger: BooleanSupplier): Command =
    roller.outtake().until(retractTrigger).andThen(retractIntake())

fun farOuttakeAlgae(retractTrigger: BooleanSupplier): Command =
    roller.farOuttake().until(retractTrigger).andThen(retractIntake())

fun retractIntake(): Command = extender.retract()
