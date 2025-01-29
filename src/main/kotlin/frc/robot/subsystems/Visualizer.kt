package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
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

private val INITIAL_Elevator_1_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.14345040))

private val INITIAL_Elevator_2_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.20545040))

private val INITIAL_CLIMBER_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.24), z = Meters.of(0.285))
private val kWheelRadius = Meters.of(0.0508)

class Visualizer {
    private val swerveTurnAngle = frc.robot.swerveDrive.SwerveTurnAngle
    private val swerveDriveAngle = frc.robot.swerveDrive.SwerveDriveAngle

    private val climbAngle = frc.robot.climber.angle.invoke()

    private val elevatorHeight = frc.robot.elevator.height

    private val extenderPosition = frc.robot.extender.position.invoke()
    val wristAngle = frc.robot.wrist.angle.invoke()
    private fun getElevatorPoses(): Pair<Pose3d, Pose3d> {

        val secondStageHeight = elevatorHeight.invoke().`in`(Meters)
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

    private fun getSwerveModulePoseTurn(
        moduleX: Double,
        moduleY: Double,
        moduleYaw: Angle
    ): Pose3d {
        return Pose3d(
            Translation3d(moduleX, moduleY, kWheelRadius.`in`(Meters)),
            getRotation3d(yaw = moduleYaw)
        )
    }

    private fun getSwerveModulePoseDrive(
        moduleX: Double,
        moduleY: Double,
        moduleYaw: Angle,
        modulePitch: Angle
    ): Pose3d {

        return Pose3d(
            Translation3d(moduleX, moduleY, kWheelRadius.`in`(Meters)),
            getRotation3d(yaw = moduleYaw, pitch = modulePitch)
        )
    }

    private fun getAllSwerveModulePoseTurn(): Array<Pose3d> {
        val swervePosesTurn: Array<Pose3d> =
            arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
        for (i in 0..3) {
            swervePosesTurn[i] =
                getSwerveModulePoseTurn(
                    swerveMoudlePose[i].x,
                    swerveMoudlePose[i].y,
                    swerveTurnAngle[i]
                )
        }
        return swervePosesTurn
    }

    private fun getAllSwerveModulePoseDrive(): Array<Pose3d> {
        val swervePosesDrive: Array<Pose3d> =
            arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())

        for (i in 0..3) {
            swervePosesDrive[i] =
                getSwerveModulePoseDrive(
                    swerveMoudlePose[i].x,
                    swerveMoudlePose[i].y,
                    swerveTurnAngle[i],
                    swerveDriveAngle[i]
                )
        }
        return swervePosesDrive
    }

    @AutoLogOutput
    fun getSubsystemsPoses(): Array<Pose3d> {
        val swervePosesTurn = getAllSwerveModulePoseTurn()
        val swervePosesDrive = getAllSwerveModulePoseDrive()

        val intakePose =
            getPose3d(
                INITIAL_INTAKE_TRANSLATION +
                    (getTranslation3d(x = extenderPosition))
            )
        val intakeRollerPose =
            getPose3d(
                INITIAL_INTAKE_Roller_TRANSLATION +
                    getTranslation3d(x = extenderPosition)
            )

        val (firstStagePose, secondStagePose) = getElevatorPoses()
        val wristPose =
            getPose3d(
                getTranslation3d(
                    x = INITIAL_WRIST_TRANSLATION.x,
                    y = INITIAL_WRIST_TRANSLATION.y,
                    z =
                        INITIAL_WRIST_TRANSLATION.z +
                            elevatorHeight.invoke().`in`(Meters)
                ),
                getRotation3d(pitch = wristAngle)
            )

        val coralRollersPoseDown = wristPose
        val coralRollersPoseUp = wristPose

        val algaeRemoverPose =
            getPose3d(
                x = INITIAL_ALGEA_REMOVER_ROLLER_TRANSLATION.x,
                z =
                    INITIAL_ALGEA_REMOVER_ROLLER_TRANSLATION.z +
                        elevatorHeight.invoke().`in`(Meters),
            )

        val climberPose =
            getPose3d(
                translation = INITIAL_CLIMBER_TRANSLATION,
                rotation = getRotation3d(pitch = climbAngle)
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
