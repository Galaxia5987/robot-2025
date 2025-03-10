package frc.robot.lib.extensions

import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.LogTable

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
    defaultValue: T,
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

fun CommandGenericHID.rumble(): Command {
    return Commands.startEnd(
        { hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
        { hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) })
}

fun (() -> Boolean).debounce(time: Double): Trigger = Trigger(this).debounce(time)
fun (() -> Boolean).debounce(time: Time): Trigger = Trigger(this).debounce(time.`in`(Seconds))

infix fun Command.andThen(command: Command): SequentialCommandGroup = andThen(command)
