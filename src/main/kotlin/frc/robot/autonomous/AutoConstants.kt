package frc.robot.autonomous

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Filesystem
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

val ALIGNMENT_POSES = parseChoreoPoses()

val INTAKE_FEEDER_DISTANCE: Distance = Units.Meters.of(0.0)
val INTAKE_FEEDER_TIME: Time = Units.Seconds.of(3.0)
val SCORE_DISTANCE: Distance = Units.Meters.of(0.0)
val SCORE_TIME: Time = Units.Seconds.of(5.0)
val AUTO_DISTANCE_TOLERANCE: Distance = Units.Centimeters.of(2.0)
