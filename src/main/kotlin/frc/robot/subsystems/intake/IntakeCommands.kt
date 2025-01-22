package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.extender
import frc.robot.roller

fun intake(): Command = extender.extend().alongWith(roller.intake())

fun outtake(): Command = roller.outtake()

fun farOuttake(): Command = roller.farOuttake()

fun retract(): Command = extender.retract()
