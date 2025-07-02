package frc.robot.lib

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WrapperCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import kotlin.math.hypot
import org.littletonrobotics.junction.LogTable

fun ChassisSpeeds.getSpeed() = hypot(vxMetersPerSecond, vyMetersPerSecond)

fun List<Any>.toDoubleArray(): DoubleArray {
    return this.map { it as Double }.toTypedArray().toDoubleArray()
}

fun List<Any>.toIntArray(): IntArray {
    return this.map { it as Int }.toTypedArray().toIntArray()
}

fun List<Any>.toBooleanArray(): BooleanArray {
    return this.map { it as Boolean }.toTypedArray().toBooleanArray()
}

fun LogTable.put(key: String, defaultValue: List<Any>) {
    when {
        defaultValue.all { it is Double } ->
            put(key, defaultValue.toDoubleArray())
        defaultValue.all { it is Int } -> put(key, defaultValue.toIntArray())
        defaultValue.all { it is Boolean } ->
            put(key, defaultValue.toBooleanArray())
        else ->
            throw IllegalArgumentException(
                "Unsupported List type: ${defaultValue::class.simpleName}"
            )
    }
}

inline fun <reified T : List<Any>> LogTable.get(
    key: String,
    defaultValue: T
): T {
    val type = defaultValue::class

    val result: List<Any> =
        when {
            defaultValue.all { it is Double } ->
                get(key, defaultValue.toDoubleArray()).toList()
            defaultValue.all { it is Int } ->
                get(key, defaultValue.toIntArray()).toList()
            defaultValue.all { it is Boolean } ->
                get(key, defaultValue.toBooleanArray()).toList()
            else ->
                throw IllegalArgumentException(
                    "Unable to LogTable.get List of type: ${type.simpleName}"
                )
        }
    return if (T::class == MutableList::class) result.toMutableList() as T
    else result as T
}

fun Command.handleInterrupt(command: Command): WrapperCommand =
    handleInterrupt {
        command.schedule()
    }

fun Command.finallyDo(command: Command): WrapperCommand =
    finallyDo(
        Runnable {
            this.cancel()
            if (command.isScheduled) command.cancel()
            command.schedule()
        }
    )

fun CommandXboxController.setRumble(strength: Double) {
    this.hid.setRumble(GenericHID.RumbleType.kBothRumble, strength)
}

fun CommandXboxController.rumbleCommand(): Command {
    return Commands.startEnd({ this.setRumble(1.0) }, { this.setRumble(0.0) })
}
