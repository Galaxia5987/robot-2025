package frc.robot.autonomous

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.IS_RED
import frc.robot.lib.flip
import frc.robot.lib.flipIfNeeded
import frc.robot.lib.getTranslation2d
import frc.robot.lib.mirror
import java.io.File
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive

val REEF_RADIUS: Distance = Units.Meters.of(0.8317)

val ROBOT_SIDE_LENGTH = Units.Meters.of(0.825)

// If measured on the red side should flip.
// Two field measurements for finding the reef center. The robot should touch the reef.
val ReefFaceLeft: Pose2d = Pose2d(14.32, 3.86, Rotation2d.k180deg).flip()
val ReefFaceRight: Pose2d = Pose2d(14.32, 4.20, Rotation2d.k180deg).flip()

// 4.48945, FIELD_WIDTH / 2
// The calculated center of the reef, used for calculating all other scoring positions.
val ReefCenter = getTranslation2d(4.48945, FlippingUtil.fieldSizeY / 2)
//    getTranslation2d(
//        (ReefFaceLeft.x + ReefFaceRight.x) / 2 +
//            REEF_RADIUS.`in`(Units.Meters) +
//            ROBOT_SIDE_LENGTH.`in`(Units.Meters) / 2,
//        (ReefFaceLeft.y + ReefFaceRight.y) / 2
//    )

val Reef4Left: Pose2d =
    ReefFaceLeft.plus(
        Transform2d(
            -Units.Centimeters.of(4.0),
            Units.Centimeters.zero(),
            Rotation2d.kZero
        )
    )
val Reef4Right: Pose2d =
    ReefFaceRight.plus(
        Transform2d(
            -Units.Centimeters.of(4.0),
            Units.Centimeters.zero(),
            Rotation2d.kZero
        )
    )

val Reef5Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(60.0))
val Reef5Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(60.0))

val Reef6Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(120.0))
val Reef6Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(120.0))

val Reef1Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(180.0))
val Reef1Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(180.0))

val Reef2Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(240.0))
val Reef2Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(240.0))

val Reef3Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(300.0))
val Reef3Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(300.0))

val FeederRight: Pose2d = Pose2d(1.5, 0.9, Rotation2d.fromDegrees(-125.0))
val FeederLeft: Pose2d = Pose2d(1.5, 7.1, Rotation2d.fromDegrees(125.0))

val buttonToPoseAndTagMap =
    mapOf(
        9 to
            Pair({ Reef4Left.flipIfNeeded() }, { if (IS_RED) 7 else 18 }), // L1
        10 to
            Pair(
                { Reef4Right.flipIfNeeded() },
                { if (IS_RED) 7 else 18 }
            ), // R1
        8 to
            Pair(
                { Reef3Right.flipIfNeeded() },
                { if (IS_RED) 6 else 19 }
            ), // L2
        11 to
            Pair({ Reef5Left.flipIfNeeded() }, { if (IS_RED) 8 else 17 }), // R2
        4 to
            Pair({ Reef3Left.flipIfNeeded() }, { if (IS_RED) 6 else 19 }), // L3
        12 to
            Pair(
                { Reef5Right.flipIfNeeded() },
                { if (IS_RED) 8 else 17 }
            ), // R3
        7 to
            Pair(
                { Reef2Right.flipIfNeeded() },
                { if (IS_RED) 11 else 20 }
            ), // L4
        1 to
            Pair({ Reef6Left.flipIfNeeded() }, { if (IS_RED) 9 else 22 }), // R4
        6 to
            Pair(
                { Reef2Left.flipIfNeeded() },
                { if (IS_RED) 11 else 20 }
            ), // L5
        2 to
            Pair(
                { Reef6Right.flipIfNeeded() },
                { if (IS_RED) 9 else 22 }
            ), // R5
        5 to
            Pair(
                { Reef1Right.flipIfNeeded() },
                { if (IS_RED) 10 else 21 }
            ), // L6
        3 to
            Pair({ Reef1Left.flipIfNeeded() }, { if (IS_RED) 10 else 21 }) // R6
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

val X_ALIGNMENT_TOLERANCE: Distance = Units.Centimeters.of(2.75)
val Y_ALIGNMENT_TOLERANCE: Distance = Units.Centimeters.of(2.75)
val ROTATIONAL_ALIGNMENT_TOLERANCE: Angle = Units.Degrees.of(1.0)
val ALIGNMENT_ELEVATOR_MAX_DISTANCE: Distance = Units.Meters.of(0.4)
val ALIGNMENT_ELEVATOR_MIN_DISTANCE: Distance = Units.Meters.of(0.0)
val PATH_FIND_END_VELOCITY: LinearVelocity = Units.MetersPerSecond.of(0.5)
val NET_ZONE: Rectangle2d =
    Rectangle2d(Translation2d(7.3, 8.0), Translation2d(7.73, 4.4))
val OPPOSING_NET_ZONE = NET_ZONE.mirror()
val NET_ZONES = listOf(NET_ZONE, OPPOSING_NET_ZONE)
val MOVE_WRIST_UP_RADIUS: Distance = Units.Meters.of(2.0)
val MOVE_WRIST_DOWN_RADIUS: Distance = Units.Meters.of(2.2)
