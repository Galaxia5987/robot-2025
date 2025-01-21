package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller

class IntakeCommands(
    private val extender: Extender,
    private val roller: Roller
) {
    fun intake(): Command = extender.extend().alongWith(roller.intake())

    fun outtake(): Command =
        roller.outtake()

    fun farOuttake(): Command =
        roller.farOuttake()

    fun retract(): Command = extender.retract()
}
