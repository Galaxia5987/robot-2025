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
val ReefFaceLeft: Pose2d = Pose2d(14.31, 3.86, Rotation2d.k180deg).flip()
val ReefFaceRight: Pose2d = Pose2d(14.31, 4.18, Rotation2d.k180deg).flip()
val ReefFaceMiddle: Pose2d = Pose2d(14.31, 4.01, Rotation2d.k180deg).flip()

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
val Reef4Middle: Pose2d = ReefFaceMiddle

val Reef5Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(60.0))
val Reef5Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(60.0))
val Reef5Middle: Pose2d =
    Reef4Middle.rotateAround(ReefCenter, Rotation2d.fromDegrees(60.0))

val Reef6Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(120.0))
val Reef6Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(120.0))
val Reef6Middle: Pose2d =
    Reef4Middle.rotateAround(ReefCenter, Rotation2d.fromDegrees(120.0))

val Reef1Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(180.0))
val Reef1Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(180.0))
val Reef1Middle: Pose2d =
    Reef4Middle.rotateAround(ReefCenter, Rotation2d.fromDegrees(180.0))

val Reef2Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(240.0))
val Reef2Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(240.0))
val Reef2Middle: Pose2d =
    Reef4Middle.rotateAround(ReefCenter, Rotation2d.fromDegrees(240.0))

val Reef3Left: Pose2d =
    Reef4Left.rotateAround(ReefCenter, Rotation2d.fromDegrees(300.0))
val Reef3Right: Pose2d =
    Reef4Right.rotateAround(ReefCenter, Rotation2d.fromDegrees(300.0))
val Reef3Middle: Pose2d =
    Reef4Middle.rotateAround(ReefCenter, Rotation2d.fromDegrees(300.0))

val FeederRight: Pose2d = Pose2d(1.5, 0.9, Rotation2d.fromDegrees(-125.0))
val FeederLeft: Pose2d = Pose2d(1.5, 7.1, Rotation2d.fromDegrees(125.0))

val buttonToPoseAndTagMap =
    mapOf(
        9 to
            Triple(
                { Reef4Left.flipIfNeeded() },
                { if (IS_RED) 7 else 18 },
                { Reef4Middle.flipIfNeeded() }
            ), // L1
        10 to
            Triple(
                { Reef4Right.flipIfNeeded() },
                { if (IS_RED) 7 else 18 },
                { Reef4Middle.flipIfNeeded() }
            ), // R1
        8 to
            Triple(
                { Reef3Right.flipIfNeeded() },
                { if (IS_RED) 6 else 19 },
                { Reef3Middle.flipIfNeeded() }
            ), // L2
        11 to
            Triple(
                { Reef5Left.flipIfNeeded() },
                { if (IS_RED) 8 else 17 },
                { Reef5Middle.flipIfNeeded() }
            ), // R2
        4 to
            Triple(
                { Reef3Left.flipIfNeeded() },
                { if (IS_RED) 6 else 19 },
                { Reef3Middle.flipIfNeeded() }
            ), // L3
        12 to
            Triple(
                { Reef5Right.flipIfNeeded() },
                { if (IS_RED) 8 else 17 },
                { Reef5Middle.flipIfNeeded() }
            ), // R3
        7 to
            Triple(
                { Reef2Right.flipIfNeeded() },
                { if (IS_RED) 11 else 20 },
                { Reef2Middle.flipIfNeeded() }
            ), // L4
        1 to
            Triple(
                { Reef6Left.flipIfNeeded() },
                { if (IS_RED) 9 else 22 },
                { Reef6Middle.flipIfNeeded() }
            ), // R4
        6 to
            Triple(
                { Reef2Left.flipIfNeeded() },
                { if (IS_RED) 11 else 20 },
                { Reef2Middle.flipIfNeeded() }
            ), // L5
        2 to
            Triple(
                { Reef6Right.flipIfNeeded() },
                { if (IS_RED) 9 else 22 },
                { Reef6Middle.flipIfNeeded() }
            ), // R5
        5 to
            Triple(
                { Reef1Right.flipIfNeeded() },
                { if (IS_RED) 10 else 21 },
                { Reef1Middle.flipIfNeeded() }
            ), // L6
        3 to
            Triple(
                { Reef1Left.flipIfNeeded() },
                { if (IS_RED) 10 else 21 },
                { Reef1Middle.flipIfNeeded() }
            ) // R6
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
val X_LENIENT_TOLERANCE: Distance = Units.Centimeters.of(5.5)
val Y_LENIENT_TOLERANCE: Distance = Units.Centimeters.of(5.5)
val ROTATIONAL_ALIGNMENT_TOLERANCE: Angle = Units.Degrees.of(1.0)
val ALIGNMENT_ELEVATOR_MAX_DISTANCE: Distance = Units.Meters.of(0.4)
val ALIGNMENT_ELEVATOR_MIN_DISTANCE: Distance = Units.Meters.of(0.0)
val PATH_FIND_END_VELOCITY: LinearVelocity = Units.MetersPerSecond.of(0.5)
val NET_ZONE: Rectangle2d =
    Rectangle2d(Translation2d(7.0, 8.0), Translation2d(7.73, 4.4))
val OPPOSING_NET_ZONE = NET_ZONE.flip().mirror()
val NET_ZONES = listOf(NET_ZONE, OPPOSING_NET_ZONE)
val MOVE_WRIST_UP_RADIUS: Distance = Units.Meters.of(2.0)
val MOVE_WRIST_DOWN_RADIUS: Distance = Units.Meters.of(2.2)
