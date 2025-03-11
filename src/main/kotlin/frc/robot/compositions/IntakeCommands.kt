package frc.robot.compositions

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.extender
import frc.robot.lib.extensions.then
import frc.robot.lib.extensions.until
import frc.robot.lib.extensions.withName
import frc.robot.roller
import java.util.function.BooleanSupplier

fun intakeAlgae(): Command =
    extender.extend().alongWith(roller.intake()).withName("Intake/Intake Algae")

fun outtakeAlgae(retractTrigger: BooleanSupplier): Command =
    roller.outtake() until retractTrigger then extender.retract() withName "Intake/Outtake Algae"
