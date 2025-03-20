package frc.robot.compositions.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.IS_RED
import frc.robot.lib.extensions.*
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.getTransform2d
import frc.robot.lib.getTranslation2d
import kotlinx.serialization.json.*
import java.io.File

private val REFERENCE_REEF_FACE = run {
    val reefRadius: Distance = 0.8317.m
    val robotSideLength: Distance = 0.825.m
    // If measured on the red side should flip.
    // Two field measurements for finding the reef center. The robot should touch the reef.
    val reefLeftFaceMiddle = Translation2d(14.34.m, 3.84.m).flip()
    val reefRightFaceMiddle = Translation2d(14.34.m, 4.19.m).flip()
    // The calculated center of the reef, used for calculating all other scoring positions.
    val reefCenter = (reefRightFaceMiddle + reefLeftFaceMiddle) / 2

    reefCenter + getTranslation2d(x = reefRadius + robotSideLength / 2)
}

val Reef4Left: Pose2d =
    REEF_LEFT_FACE_MIDDLE + Transform2d(
        -4.cm,
        Units.Centimeters.zero(),
        Rotation2d.kZero
    )
val Reef4Right: Pose2d =
    REEF_RIGHT_FACE_MIDDLE + Transform2d(
        -4.cm,
        Units.Centimeters.zero(),
        Rotation2d.kZero
    )

val Reef5Left: Pose2d =
    Reef4Left.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(60.0))
val Reef5Right: Pose2d =
    Reef4Right.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(60.0))

val Reef6Left: Pose2d =
    Reef4Left.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(120.0))
val Reef6Right: Pose2d =
    Reef4Right.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(120.0))

val Reef1Left: Pose2d =
    Reef4Left.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(180.0))
val Reef1Right: Pose2d =
    Reef4Right.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(180.0))

val Reef2Left: Pose2d =
    Reef4Left.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(240.0))
val Reef2Right: Pose2d =
    Reef4Right.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(240.0))

val Reef3Left: Pose2d =
    Reef4Left.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(300.0))
val Reef3Right: Pose2d =
    Reef4Right.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(300.0))

val FeederRightMidPose: Pose2d =
    Pose2d(2.563, 1.647, Rotation2d.fromDegrees(-120.0))
val FeederRight: Pose2d = Pose2d(1.635, 1.467, Rotation2d.fromDegrees(-130.0))
val FeederLeftMidPose: Pose2d =
    Pose2d(3.520, 6.353, Rotation2d.fromDegrees(120.0))
val FeederLeft: Pose2d = Pose2d(1.197, 7.031, Rotation2d.fromDegrees(120.0))

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

val X_ALIGNMENT_TOLERANCE: Distance = 2.75.cm
val Y_ALIGNMENT_TOLERANCE: Distance = 2.75.cm
val ROTATIONAL_ALIGNMENT_TOLERANCE: Angle = 1.deg
val ALIGNMENT_ELEVATOR_MAX_DISTANCE: Distance = 40.cm
val ALIGNMENT_ELEVATOR_MIN_DISTANCE: Distance = 18.cm
