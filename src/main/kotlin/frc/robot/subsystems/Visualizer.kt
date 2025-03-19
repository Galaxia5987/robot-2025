package frc.robot.subsystems

import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import frc.robot.*
import frc.robot.lib.*
import frc.robot.subsystems.drive.Drive
import kotlin.math.cos
import kotlin.math.sin
import org.littletonrobotics.junction.AutoLogOutput
import java.lang.reflect.Field
import java.lang.reflect.Modifier

private val swerveModulePose: Array<Translation2d> =
    Drive.getModuleTranslations()

private val INITIAL_INTAKE_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.32), z = Meters.of(0.35))
private val INITIAL_INTAKE_Roller_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.32 + 0.62890), z = Meters.of(0.35))

private val INITIAL_WRIST_TRANSLATION =
    getTranslation3d(x = Meters.of(0.20715), z = Meters.of(0.995))

private val INITIAL_Elevator_1_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.14345040))

private val INITIAL_Elevator_2_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.20545040))

private val INITIAL_CLIMBER_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.3), z = Meters.of(0.354))
private val kWheelRadius = Meters.of(0.0508)
private val CORAL_ROLLER_UP_C2C: Array<Double> =
    arrayOf(0.31500, 3.21) // arrayOf(C2C Distance, Angle (in rad))
private val CORAL_ROLLER_DOWN_C2C: Array<Double> =
    arrayOf(0.14203, 3.5) // arrayOf(C2C Distance, Angle (in rad))

private val CORAL_C2C: Array<Double> =
    arrayOf(0.24, 3.5) // arrayOf(C2C Distance, Angle (in rad))

private val WRIST_ANGLE_OFFSET = Degrees.of(90.0)

private val CLIMBER_ANGLE_OFFSET = Degrees.of(135.0)

private val CORAL_ANGLE_OFFSET = Degrees.of(-10.0)

private val ALGAE_OFFSET = Transform3d(0.4, 0.0, 0.35, Rotation3d())

class Visualizer {
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

    @AutoLogOutput(key = "Visualizer/AlgaePoseInRobot")
    private fun getAlgaePose(): Pose3d? =
        if (CURRENT_MODE != Mode.REAL && roller.shouldVisualizeAlgaeInSim())
            driveSimulation
                ?.simulatedDriveTrainPose
                ?.toPose3d()
                ?.plus(ALGAE_OFFSET)
        else Pose3d()

    @AutoLogOutput(key = "Visualizer/CoralPoseInRobot")
    private fun getCoralPose(): Pose3d? =
        if (CURRENT_MODE != Mode.REAL && gripper.hasCoral.asBoolean)
            driveSimulation
                ?.simulatedDriveTrainPose
                ?.toPose3d()
                ?.plus(
                    getGripperRollerPoseInverted(
                        CORAL_C2C[0],
                        CORAL_C2C[1]
                    ).toTransform()
                )
                ?.withRotation(
                    pitch = -wrist.angle.invoke() + CORAL_ANGLE_OFFSET,
                    yaw =
                    driveSimulation.simulatedDriveTrainPose.rotation
                        .measure!!
                )
        else Pose3d()

    fun getCoralPoseRelativeToRobot(): Pose3d = getGripperRollerPoseInverted(CORAL_C2C[0], CORAL_C2C[1])

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

    private fun getAdjustedWristAngle(angleFromCenter: Double): Double =
        angleFromCenter - wrist.angle.invoke().`in`(Radians) +
                WRIST_ANGLE_OFFSET.`in`(Radians)

    private fun getGripperRollerPose(
        distanceFromCenter: Double = 0.0,
        angleFromCenter: Double = 0.0
    ): Pose3d {
        return getPose3d(
            getTranslation3d(
                x =
                INITIAL_WRIST_TRANSLATION.x -
                        cos(getAdjustedWristAngle(angleFromCenter)) *
                        distanceFromCenter,
                z =
                (INITIAL_WRIST_TRANSLATION.z +
                        sin(getAdjustedWristAngle(angleFromCenter)) *
                        distanceFromCenter +
                        elevator.height.invoke().`in`(Meters))
            ),
            getRotation3d(pitch = gripper.rollerAngle.`in`(Rotations))
        )
    }

    private fun getGripperRollerPoseInverted(
        distanceFromCenter: Double = 0.0,
        angleFromCenter: Double = 0.0
    ): Pose3d {
        return getPose3d(
            getTranslation3d(
                x =
                INITIAL_WRIST_TRANSLATION.x -
                        cos(getAdjustedWristAngle(angleFromCenter)) *
                        distanceFromCenter,
                z =
                (INITIAL_WRIST_TRANSLATION.z +
                        sin(getAdjustedWristAngle(angleFromCenter)) *
                        distanceFromCenter +
                        elevator.height.invoke().`in`(Meters))
            ),
            getRotation3d(pitch = -gripper.rollerAngle.`in`(Rotations))
        )
    }

    @AutoLogOutput
    fun getSubsystemsPoses(): Array<Pose3d> {
        val swervePosesTurn = getAllSwerveModulePoseTurn()
        val swervePosesDrive = getAllSwerveModulePoseDrive()
        var extenderPosition = extender.position.invoke()
        if (extenderPosition < Meters.zero()) {
            extenderPosition = Meters.zero()
        }
        val intakePose =
            getPose3d(
                INITIAL_INTAKE_TRANSLATION +
                        (getTranslation3d(x = extenderPosition))
            )
        val intakeRollerPose =
            getPose3d(
                INITIAL_INTAKE_Roller_TRANSLATION +
                        getTranslation3d(x = extenderPosition),
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
                getRotation3d(
                    pitch = -wrist.angle.invoke() + WRIST_ANGLE_OFFSET
                )
            )

        val coralRollersPoseDown =
            getGripperRollerPoseInverted(
                CORAL_ROLLER_DOWN_C2C[0],
                CORAL_ROLLER_DOWN_C2C[1]
            )
        val coralRollersPoseUp =
            getGripperRollerPose(CORAL_ROLLER_UP_C2C[0], CORAL_ROLLER_UP_C2C[1])

        val climberPose =
            getPose3d(
                translation = INITIAL_CLIMBER_TRANSLATION,
                rotation = getRotation3d(pitch = climber.angle.invoke() + CLIMBER_ANGLE_OFFSET)
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
            climberPose
        )
    }
}
