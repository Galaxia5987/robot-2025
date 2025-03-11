// Copyright FRC 1657
// https://github.com/Hamosad1657/HamosadLib/blob/main/src/main/kotlin/com/hamosad1657/lib/commands/Extentions.kt

package frc.robot.lib.extensions

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.BooleanSupplier

infix fun Command.until(condition: BooleanSupplier): Command = until(condition)
infix fun Command.then(next: Command): Command = andThen(next)
infix fun Command.then(next: () -> Command): Command = andThen(next())
infix fun Command.finallyDo(end: (interrupted: Boolean) -> Unit): Command = finallyDo(end)
infix fun Command.finallyDo(command: Command): Command = finallyDo { _ -> command.schedule() }

infix fun Command.alongWith(parallel: Command): Command = alongWith(parallel)
infix fun Command.raceWith(parallel: Command): Command = raceWith(parallel)

infix fun Command.withTimeout(seconds: Double): Command = withTimeout(seconds)

/**
 * Good for multi-subsystem commands.
 * For single-subsystem commands, use [SubsystemBase.withName].
 */
infix fun Command.withName(commandName: String): Command {
    val prefix = if (requirements.count() == 1) requirements.first().name else ""
    return this.withName(name)
}


/**
 * Good for multi-subsystem commands.
 * For single-subsystem commands, use [SubsystemBase.withName].
 */
fun withName(commandName: String, commandSupplier: () -> Command): Command =
    commandSupplier().also { it.name = commandName }

/**
 * Good for single-subsystem commands.
 * Appends the name of the subsystem to the String in [commandName : subsystemName] format.
 *
 * For multi-subsystem commands, use [withName].
 */
fun SubsystemBase.withName(commandName: String, command: Command): Command =
    command withName "$name/$commandName"

fun Trigger.onTrue(action: () -> Unit) = onTrue(Commands.runOnce(action))
fun Trigger.onFalse(action: () -> Unit) = onFalse(Commands.runOnce(action))