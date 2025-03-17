package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.extender
import frc.robot.roller
import java.util.function.BooleanSupplier

fun intakeAlgae(): Command =
    extender.extend().alongWith(roller.intake()).withName("Intake/Intake Algae")

fun outtakeAlgae(retractTrigger: BooleanSupplier): Command =
    roller
        .outtake()
        .until(retractTrigger)
        .andThen(extender.retract())
        .withName("Intake/Outtake Algae")

fun intakeToGripper(): Command =
    extender.passToGripper().alongWith(roller.slowIntake())