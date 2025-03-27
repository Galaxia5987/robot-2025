package frc.robot.compositions.autonomous

import edu.wpi.first.math.geometry.*
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


val FACE_NAMES_TO_POSES = run {
    val robotSideLength: Distance = 0.825.m
    val reefCenter = Translation2d(3.2, 4.015)
    val reefRadius: Distance = Meters.of(0.8317)
    val reefArea = Ellipse2d(Translation2d(3.2, 4.015), reefRadius)
    val destination = selectedReefPose.first
    val isIntersectingReef =
        (1..20).any { reefArea.contains(robotPose.interpolate(destination, it * 0.05).translation) }
    val referenceReefFace = reefCenter - getTranslation2d(x = reefRadius + robotSideLength / 2)
    val transformFromFaceCenterToBranch = getTransform2d(y = 4.cm)

    fun getScorePose(angle: Rotation2d, transform: Transform2d) =
        Pose2d(referenceReefFace.rotateAround(reefCenter, angle), angle) + transform

    val feederPose = Pose2d(1.5987.m, 0.8.m, -125.deg.toRotation2d())

    (1..12).flatMap { number ->
        val angle = (60 * (number - 1)).deg.toRotation2d()
        listOf(
            "${number}R" to getScorePose(angle, -transformFromFaceCenterToBranch),
            "${number}L" to getScorePose(-angle, transformFromFaceCenterToBranch)
        )
    }.toMap() + mapOf(
        "FR" to feederPose, "FL" to feederPose.mirror()
    )
}

val FeederRightMidPose: Pose2d = Pose2d(2.563, 1.647, Rotation2d.fromDegrees(-120.0))
val FeederRight: Pose2d = Pose2d(1.635, 1.467, Rotation2d.fromDegrees(-130.0))
val FeederLeftMidPose: Pose2d = Pose2d(3.520, 6.353, Rotation2d.fromDegrees(120.0))
val FeederLeft: Pose2d = Pose2d(1.197, 7.031, Rotation2d.fromDegrees(120.0))

val buttonToPoseAndTagMap = mapOf(
    9 to Pair(Reef4Left::flipIfNeeded, { if (IS_RED) 7 else 18 }), // L1
    10 to Pair(Reef4Right::flipIfNeeded) { if (IS_RED) 7 else 18 }, // R1
    8 to Pair(
        Reef3Right::flipIfNeeded, { if (IS_RED) 6 else 19 }), // L2
    11 to Pair(Reef5Left::flipIfNeeded) { if (IS_RED) 8 else 17 }, // R2
    4 to Pair(Reef3Left::flipIfNeeded, { if (IS_RED) 6 else 19 }), // L3
    12 to Pair(
        Reef5Right::flipIfNeeded, { if (IS_RED) 8 else 17 }), // R3
    7 to Pair(
        Reef2Right::flipIfNeeded, { if (IS_RED) 11 else 20 }), // L4
    1 to Pair(Reef6Left::flipIfNeeded, { if (IS_RED) 9 else 22 }), // R4
    6 to Pair(
        Reef2Left::flipIfNeeded, { if (IS_RED) 11 else 20 }), // L5
    2 to Pair(
        Reef6Right::flipIfNeeded, { if (IS_RED) 9 else 22 }), // R5
    5 to Pair(
        Reef1Right::flipIfNeeded, { if (IS_RED) 10 else 21 }), // L6
    3 to Pair(Reef1Left::flipIfNeeded, { if (IS_RED) 10 else 21 }) // R6
)

private fun getValueFromJson(element: JsonElement, valName: String): Double =
    element.jsonObject[valName]?.jsonObject?.get("val")?.jsonPrimitive?.doubleOrNull ?: throw IllegalArgumentException(
        "Trying to load non-existent Choreo pose"
    )

private fun parseChoreoPoses(): Map<String, Pose2d> {
    val json = Json { ignoreUnknownKeys = true }
    val jsonObject = json.parseToJsonElement(
        File(
            "${Filesystem.getDeployDirectory()}/choreo/autotrajectories/AutoPaths.chor"
        ).readText()
    ).jsonObject

    val jsonPoses = jsonObject["variables"]?.jsonObject?.get("poses")?.jsonObject ?: throw IllegalArgumentException(
        "Trying to load non-existent Choreo variables"
    )

    return jsonPoses.mapValues { (key, pose) ->
        Pose2d(
            getValueFromJson(pose, "x"), getValueFromJson(pose, "y"), Rotation2d(getValueFromJson(pose, "heading"))
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
