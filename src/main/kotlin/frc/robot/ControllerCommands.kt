package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

fun startRumble(controller: CommandXboxController): Command {
    return Commands.runOnce(
        {
            controller.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
        }
    )
}

fun stopRumble(controller: CommandXboxController): Command {
    return Commands.runOnce(
        {
            controller.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        }
    )
}