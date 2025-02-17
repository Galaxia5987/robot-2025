package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.lib.Gains
import frc.robot.lib.flipIfNeeded
import java.io.File
import kotlinx.serialization.json.*

private fun getValueFromJson(element: JsonElement, valName: String): Double =
    element.jsonObject[valName]
        ?.jsonObject
        ?.get("val")
        ?.jsonPrimitive
        ?.doubleOrNull
        ?: throw IllegalArgumentException(
            "Trying to load non-existent Choreo pose"
        )

private fun parseChoreoPoses(): Map<String, Pose2d> {
    val json = Json { ignoreUnknownKeys = true }
    val jsonObject =
        json
            .parseToJsonElement(
                File(
                        "${Filesystem.getDeployDirectory()}/choreo/autotrajectories/AutoPaths.chor"
                    )
                    .readText()
            )
            .jsonObject

    val jsonPoses =
        jsonObject["variables"]?.jsonObject?.get("poses")?.jsonObject
            ?: throw IllegalArgumentException(
                "Trying to load non-existent Choreo variables"
            )

    return jsonPoses.mapValues { (key, pose) ->
        Pose2d(
            getValueFromJson(pose, "x"),
            getValueFromJson(pose, "y"),
            Rotation2d(getValueFromJson(pose, "heading"))
        )
    }
}

val ALIGNMENT_POSES
    get() = parseChoreoPoses().mapValues { it.value.flipIfNeeded() }

val LINEAR_ALIGNMENT_TOLERANCE: Distance = Units.Centimeters.of(1.0)
val ROTATIONAL_ALIGNMENT_TOLERANCE: Angle = Units.Degrees.of(2.0)
val MAX_ALIGNMENT_DISTANCE: Distance = Units.Meters.of(2.0)

val ALIGNMENT_X_GAINS = Gains(5.0, 0.0, 0.2)
val ALIGNMENT_Y_GAINS = Gains(5.0, 0.0, 0.2)
val ALIGNMENT_ROTATION_GAINS = Gains(5.0, 0.0, 0.2)
