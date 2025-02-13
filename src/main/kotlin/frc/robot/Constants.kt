package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.lib.flipIfNeeded
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import org.littletonrobotics.junction.LoggedRobot
import java.io.File

const val LOOP_TIME = 0.02 // [s]
const val IS_TUNING_MODE = true

private val SPEAKER_POSE_BLUE = Translation2d(0.0, 5.5479442)

val SPEAKER_POSE: Translation2d
    get() = SPEAKER_POSE_BLUE.flipIfNeeded()

val choreoPoses: Array<Pose2d>
    get() {
        val json = Json { ignoreUnknownKeys = true }
        val jsonObject = json.parseToJsonElement(File("${Filesystem.getDeployDirectory()}/choreo/autotrajectories/AutoPaths.chor").toString()).jsonObject
        val jsonPoses = jsonObject["variables"]?.jsonObject?.get("poses")?.jsonObject ?: return Array(1, {Pose2d()})
        val poses: Array<Pose2d> = Array(jsonPoses.size, { Pose2d() })
        val i = 0
        for (pose in jsonPoses){
            poses[i] = Pose2d(
                jsonPoses[pose.key]?.jsonObject?.get("x")?.jsonObject?.get("val")?.jsonPrimitive?.doubleOrNull ?: 0.0,
                jsonPoses[pose.key]?.jsonObject?.get("y")?.jsonObject?.get("val")?.jsonPrimitive?.doubleOrNull ?: 0.0,
                Rotation2d(jsonPoses[pose.key]?.jsonObject?.get("heading")?.jsonObject?.get("val")?.jsonPrimitive?.doubleOrNull ?: 0.0))
        }
        return poses
    }

const val REEFMASTER_CANBUS_NAME = "reefmaster"
const val SWERVE_CANBUS_NAME = "swerveDrive"

val CURRENT_MODE: Mode
    get() =
        if (LoggedRobot.isReal()) {
            Mode.REAL
        } else {
            if (System.getenv()["isReplay"] == "true") {
                Mode.REPLAY
            } else {
                Mode.SIM
            }
        }
const val ALT_ROBORIO_SERIAL = ""

val ROBORIO_SERIAL_NUMBER: String
    get() = System.getenv("serialnum") ?: "Sim"

val IS_RED: Boolean
    get() =
        DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red

enum class Mode {
    REAL,
    SIM,
    REPLAY
}
