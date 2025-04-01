package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.extender
import frc.robot.roller
import java.util.function.BooleanSupplier

fun intakeAlgae(): Command =
    (extender.extend().until(extender.atSetpoint))
        .alongWith(roller.intake())
        .withName("Intake/Intake Algae")

fun outtakeAlgae(retractTrigger: BooleanSupplier): Command =
    (roller.outtake().alongWith(extender.extend()))
        .until(retractTrigger)
        .andThen(extender.retract().until(extender.atSetpoint))
        .withName("Intake/Outtake Algae")

fun intakeToGripper(): Command =
    (extender.passToGripper().until(extender.atSetpoint)).alongWith(
        roller.slowIntake()
    )
