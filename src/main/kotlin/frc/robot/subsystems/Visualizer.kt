package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Meters
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.drive.Drive
import org.littletonrobotics.junction.AutoLogOutput

private val swerveMoudlePose: Array<Translation2d> =
    Drive.getModuleTranslations()

private val INITIAL_INTAKE_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.32), z = Meters.of(0.35))
private val INITIAL_INTAKE_Roller_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.32 + 0.62890), z = Meters.of(0.35))

private val INITIAL_WRIST_TRANSLATION =
    getTranslation3d(x = Meters.of(0.27434), z = Meters.of(0.79707))

private val INITIAL_ALGEA_REMOVER_ROLLER_TRANSLATION =
    getTranslation3d(x = Meters.of(0.113), z = Meters.of(0.995))

private val INITIAL_Elevator_1_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.14345040))

private val INITIAL_Elevator_2_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.20545040))

private val INITIAL_CLIMBER_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.24), z = Meters.of(0.285))

class Visualizer {
    private val swerveDrive = frc.robot.swerveDrive
    private val climber = frc.robot.climber
    private val elevator = frc.robot.elevator
    private val gripper = frc.robot.gripper
    private val extender = frc.robot.extender
    private val roller = frc.robot.roller
    private val wrist = frc.robot.wrist
    private fun getElevatorPoses(): Pair<Pose3d, Pose3d> {
        val secondStageHeight = elevator.height.invoke().`in`(Meters)
        val firstStageHeight = secondStageHeight / 2.0

        val firstStagePose =
            getPose3d(
                x = INITIAL_Elevator_1_TRANSLATION.x,
                z = firstStageHeight.plus(INITIAL_Elevator_1_TRANSLATION.z)
            )
        val secondStagePose =
            getPose3d(
                x = INITIAL_Elevator_2_TRANSLATION.x,
                z = secondStageHeight.plus(INITIAL_Elevator_2_TRANSLATION.z)
            )
        return Pair(firstStagePose, secondStagePose)
    }

    private fun getSwerveModulePose(): Pair<Array<Pose3d>, Array<Pose3d>> {
        val swervePosesTurn: Array<Pose3d> =
            arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
        val swervePosesDrive: Array<Pose3d> =
            arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
        for (i in 0..3) {
            swervePosesTurn[i] =
                Pose3d(
                    Translation3d(
                        swerveMoudlePose[i].x,
                        swerveMoudlePose[i].y,
                        0.0508
                    ),
                    getRotation3d(yaw = swerveDrive.SwerveTurnAngle[i])
                )
            swervePosesDrive[i] =
                Pose3d(
                    Translation3d(
                        swerveMoudlePose[i].x,
                        swerveMoudlePose[i].y,
                        0.0508
                    ),
                    getRotation3d(
                        yaw = swerveDrive.SwerveTurnAngle[i],
                        pitch = swerveDrive.SwerveDriveAngle[i]
                    )
                )
        }
        return Pair(swervePosesTurn, swervePosesDrive)
    }

    @AutoLogOutput
    fun getSubsystemsPoses(): Array<Pose3d> {
        val (swervePosesTurn, swervePosesDrive) = getSwerveModulePose()

        val intakePose =
            getPose3d(
                INITIAL_INTAKE_TRANSLATION +
                    (getTranslation3d(x = extender.position.invoke()))
            )
        val intakeRollerPose =
            getPose3d(
                INITIAL_INTAKE_Roller_TRANSLATION +
                    getTranslation3d(x = extender.position.invoke())
            )

        val (firstStagePose, secondStagePose) = getElevatorPoses()
        val wristPose =
            getPose3d(
                getTranslation3d(
                    x = INITIAL_WRIST_TRANSLATION.x,
                    y = INITIAL_WRIST_TRANSLATION.y,
                    z =
                        INITIAL_WRIST_TRANSLATION.z +
                            elevator.height.invoke().`in`(Meters)
                ),
                getRotation3d(pitch = wrist.angle.invoke())
            )

        val coralRollersPoseDown = wristPose
        val coralRollersPoseUp = wristPose

        val algaeRemoverPose =
            getPose3d(
                x = INITIAL_ALGEA_REMOVER_ROLLER_TRANSLATION.x,
                z =
                    INITIAL_ALGEA_REMOVER_ROLLER_TRANSLATION.z +
                        elevator.height.invoke().`in`(Meters),
            )

        val climberPose =
            getPose3d(
                translation = INITIAL_CLIMBER_TRANSLATION,
                rotation = getRotation3d(pitch = climber.angle.invoke())
            )

        return arrayOf(
            swervePosesTurn[0],
            swervePosesDrive[0],
            swervePosesTurn[1],
            swervePosesDrive[1],
            swervePosesTurn[2],
            swervePosesDrive[2],
            swervePosesTurn[3],
            swervePosesDrive[3],
            intakePose,
            intakeRollerPose,
            firstStagePose,
            secondStagePose,
            wristPose,
            coralRollersPoseDown,
            coralRollersPoseUp,
            algaeRemoverPose,
            climberPose
        )
    }
}
