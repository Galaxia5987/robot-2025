package frc.robot.lib

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.robot.IS_RED
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

fun Translation2d.getRotationToTranslation(other: Translation2d): Rotation2d =
    (this - other).angle

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

fun Pose2d.toPose3d(): Pose3d =
    Pose3d(x, y, 0.0, Rotation3d(0.0, 0.0, rotation.radians))

fun Pose2d.flip(): Pose2d = FlippingUtil.flipFieldPose(this)

fun Pose2d.flipIfNeeded(): Pose2d = if (IS_RED) this.flip() else this

fun Translation2d.flip(): Translation2d = FlippingUtil.flipFieldPosition(this)

fun Translation2d.flipIfNeeded(): Translation2d =
    if (IS_RED) this.flip() else this

fun Rotation2d.flip(): Rotation2d = FlippingUtil.flipFieldRotation(this)

fun Rotation2d.flipIfNeeded(): Rotation2d = if (IS_RED) this.flip() else this
