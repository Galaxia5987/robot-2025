package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

private fun rumble(controller: CommandXboxController, rumbleStrength: Double): Command {
    return Commands.runOnce({
        controller.hid.setRumble(GenericHID.RumbleType.kBothRumble, rumbleStrength)
    })
}

fun startRumble(controller: CommandXboxController) = rumble(controller, 1.0)

fun stopRumble(controller: CommandXboxController) = rumble(controller, 0.0)