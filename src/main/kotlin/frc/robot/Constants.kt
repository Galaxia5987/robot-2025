package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.lib.flipIfNeeded
import java.io.File
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import org.littletonrobotics.junction.LoggedRobot

const val LOOP_TIME = 0.02 // [s]
const val IS_TUNING_MODE = true

private val SPEAKER_POSE_BLUE = Translation2d(0.0, 5.5479442)

val SPEAKER_POSE: Translation2d
    get() = SPEAKER_POSE_BLUE.flipIfNeeded()

val choreoPoses: Map<String, Pose2d>
    get() {
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
                ?: return emptyMap()

        return jsonPoses.mapValues { (key, pose) ->
            Pose2d(
                pose.jsonObject["x"]
                    ?.jsonObject
                    ?.get("val")
                    ?.jsonPrimitive
                    ?.doubleOrNull
                    ?: 0.0,
                pose.jsonObject["y"]
                    ?.jsonObject
                    ?.get("val")
                    ?.jsonPrimitive
                    ?.doubleOrNull
                    ?: 0.0,
                Rotation2d(
                    pose.jsonObject["heading"]
                        ?.jsonObject
                        ?.get("val")
                        ?.jsonPrimitive
                        ?.doubleOrNull
                        ?: 0.0
                )
            )
        }
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
