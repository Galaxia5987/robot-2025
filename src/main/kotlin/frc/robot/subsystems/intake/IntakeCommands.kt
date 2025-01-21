package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller

class IntakeCommands(private val extender: Extender, private val roller: Roller) {
    fun intake(): Command =
        extender.extend().alongWith(roller.intake())

    fun outtake(): Command =
        Commands.startEnd(roller::outtake, extender::retract, roller, extender)

    fun farOuttake(): Command =
        Commands.startEnd(roller::farOuttake, extender::retract, roller, extender)

    fun retract(): Command =
        extender.retract()
}