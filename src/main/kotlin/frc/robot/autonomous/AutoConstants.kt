package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.lib.Gains
import frc.robot.lib.flipIfNeeded
import frc.robot.lib.getPose2d
import java.io.File
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive

val Reef1: Pose2d = Pose2d(6.033, 4.000, Rotation2d.k180deg)
val Reef2: Pose2d = Pose2d(5.215, 5.266, Rotation2d.fromDegrees(-120.0))
val Reef3: Pose2d = Pose2d(3.769, 5.416, Rotation2d.fromDegrees(-60.0))
val Reef4: Pose2d = getPose2d(2.93, 4.030)
val Reef5: Pose2d = Pose2d(3.719, 2.644, Rotation2d.fromDegrees(60.0))
val Reef6: Pose2d = Pose2d(5.265, 2.664, Rotation2d.fromDegrees(120.0))
val FeederRightMidPose: Pose2d =
    Pose2d(2.563, 1.647, Rotation2d.fromDegrees(-120.0))
val FeederRight: Pose2d = Pose2d(1.635, 1.467, Rotation2d.fromDegrees(-130.0))
val FeederLeftMidPose: Pose2d =
    Pose2d(3.520, 6.353, Rotation2d.fromDegrees(120.0))
val FeederLeft: Pose2d = Pose2d(1.197, 7.031, Rotation2d.fromDegrees(120.0))

val buttonToPoseMap =
    mapOf(
        9 to Pair(Reef4, true), // L1
        10 to Pair(Reef4, false), // R1
        8 to Pair(Reef3, false), // L2
        11 to Pair(Reef5, true), // R2
        4 to Pair(Reef3, true), // L3
        12 to Pair(Reef5, false), // R3
        7 to Pair(Reef2, false), // L4
        1 to Pair(Reef6, true), // R4
        6 to Pair(Reef2, true), // L5
        2 to Pair(Reef6, false), // R5
        5 to Pair(Reef1, false), // L6
        3 to Pair(Reef1, true) // R6
    )

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

private val choreoPoses = parseChoreoPoses()
val ALIGNMENT_POSES
    get() = choreoPoses.mapValues { it.value.flipIfNeeded() }

val X_ALIGNMENT_TOLERANCE: Distance = Units.Centimeters.of(6.0)
val Y_ALIGNMENT_TOLERANCE: Distance = Units.Centimeters.of(2.0)
val ROTATIONAL_ALIGNMENT_TOLERANCE: Angle = Units.Degrees.of(2.5)
val MAX_ALIGNMENT_DISTANCE: Distance = Units.Meters.of(2.0)

val ALIGNMENT_X_GAINS = Gains(1.5, 0.0, 0.0)
val ALIGNMENT_Y_GAINS = Gains(2.5, 0.0, 0.4)
val ALIGNMENT_ROTATION_GAINS = Gains(3.0, 0.0, 0.0)

val ALIGNMENT_FORWARD_VELOCITY: LinearVelocity = Units.MetersPerSecond.of(0.5)

val X_ALIGNMENT_OFFSET = Units.Meters.of(-0.535)
val ALIGNED_Y_LEFT = -0.16
val ALIGNED_Y_RIGHT = 0.16
val ALIGNED_ROTATION = Rotation2d.fromDegrees(155.0)
