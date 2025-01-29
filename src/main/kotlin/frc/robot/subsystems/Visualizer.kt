package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.roller
import frc.robot.subsystems.drive.Drive
import kotlin.math.cos
import kotlin.math.sin
import org.littletonrobotics.junction.AutoLogOutput

private val swerveModulePose: Array<Translation2d> =
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
private val CORAL_ROLLER_UP_C2C :Array<Double> = arrayOf(0.14,0.4) //arrayOf(C2C Distance, Angle (in rad))
private val ALGAE_ROLLER_C2C :Array<Double> = arrayOf(0.25335,0.9) //arrayOf(C2C Distance, Angle (in rad))

class Visualizer {
    private val swerveDrive = frc.robot.swerveDrive

    private val climb = frc.robot.climber

    private val elevator = frc.robot.elevator

    private val extender = frc.robot.extender
    private val wrist = frc.robot.wrist

    private val gripper = frc.robot.gripper

    private fun getElevatorPoses(): Pair<Pose3d, Pose3d> {

        val secondStageHeight = elevator.height.invoke().`in`(Meters)
        val firstStageHeight = secondStageHeight / 2.0

        val firstStagePose =
            getPose3d(
                x = INITIAL_Elevator_1_TRANSLATION.x,
                z = firstStageHeight + INITIAL_Elevator_1_TRANSLATION.z
            )
        val secondStagePose =
            getPose3d(
                x = INITIAL_Elevator_2_TRANSLATION.x,
                z = secondStageHeight + INITIAL_Elevator_2_TRANSLATION.z
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
                    swerveModulePose[i].x,
                    swerveModulePose[i].y,
                    swerveDrive.SwerveTurnAngle[i]
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
                    swerveModulePose[i].x,
                    swerveModulePose[i].y,
                    swerveDrive.SwerveTurnAngle[i],
                    swerveDrive.SwerveDriveAngle[i]
                )
        }
        return swervePosesDrive
    }

    private fun getGripperRollerPose(
        distanceFromCenter: Double=0.0,
        angleFromCenter: Double=0.0
    ): Pose3d {
        return getPose3d(
            getTranslation3d(
                x =
                    INITIAL_WRIST_TRANSLATION.x -
                        cos(
                            angleFromCenter + wrist.angle.invoke().`in`(Radians)
                        ) * distanceFromCenter,
                z =
                    (INITIAL_WRIST_TRANSLATION.z +
                        sin(
                            angleFromCenter + wrist.angle.invoke().`in`(Radians)
                        ) * distanceFromCenter +
                        elevator.height.invoke().`in`(Meters))
            ),
            getRotation3d(pitch = gripper.rollerAngle.`in`(Rotations))
        )
    }
    private fun getGripperRollerPoseInverted(
        distanceFromCenter: Double=0.0,
        angleFromCenter: Double=0.0
    ): Pose3d {
        return getPose3d(
            getTranslation3d(
                x =
                    INITIAL_WRIST_TRANSLATION.x -
                        cos(
                            angleFromCenter + wrist.angle.invoke().`in`(Radians)
                        ) * distanceFromCenter,
                z =
                    (INITIAL_WRIST_TRANSLATION.z +
                        sin(
                            angleFromCenter + wrist.angle.invoke().`in`(Radians)
                        ) * distanceFromCenter +
                        elevator.height.invoke().`in`(Meters))
            ),
            getRotation3d(pitch = -gripper.rollerAngle.`in`(Rotations))
        )
    }

    @AutoLogOutput
    fun getSubsystemsPoses(): Array<Pose3d> {
        val swervePosesTurn = getAllSwerveModulePoseTurn()
        val swervePosesDrive = getAllSwerveModulePoseDrive()

        val intakePose =
            getPose3d(
                INITIAL_INTAKE_TRANSLATION +
                    (getTranslation3d(x = extender.position.invoke()))
            )
        val intakeRollerPose =
            getPose3d(
                INITIAL_INTAKE_Roller_TRANSLATION +
                    getTranslation3d(x = extender.position.invoke()),
                getRotation3d(pitch = roller.rollerAngle.`in`(Rotations))
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

        val coralRollersPoseDown = getGripperRollerPoseInverted()
        val coralRollersPoseUp = getGripperRollerPose(CORAL_ROLLER_UP_C2C[0], CORAL_ROLLER_UP_C2C[1])

        val algaeRemoverPose = getGripperRollerPoseInverted(ALGAE_ROLLER_C2C[0], ALGAE_ROLLER_C2C[1])

        val climberPose =
            getPose3d(
                translation = INITIAL_CLIMBER_TRANSLATION,
                rotation = getRotation3d(pitch = climb.angle.invoke())
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
