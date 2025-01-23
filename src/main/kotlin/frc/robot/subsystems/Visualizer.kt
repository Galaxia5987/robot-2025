package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import org.littletonrobotics.junction.AutoLogOutput

private val INITIAL_MODULE_0_Turn_TRANSLATION = getTranslation3d(x= Meters.of(0.25448),y= Meters.of(0.25448),z=Meters.of(0.0508))
private val INITIAL_MODULE_0_Drive_TRANSLATION = getTranslation3d(x= Meters.of(0.25448),y= Meters.of(0.25448),z=Meters.of(0.0508))

private val INITIAL_MODULE_1_Turn_TRANSLATION = getTranslation3d(x= Meters.of(0.25448),y= Meters.of(-0.25448),z=Meters.of(0.0508))
private val INITIAL_MODULE_1_Drive_TRANSLATION = getTranslation3d(x= Meters.of(0.25448),y= Meters.of(-0.25448),z=Meters.of(0.0508))

private val INITIAL_MODULE_2_Turn_TRANSLATION = getTranslation3d(x= Meters.of(-0.25448),y= Meters.of(0.25448),z=Meters.of(0.0508))
private val INITIAL_MODULE_2_Drive_TRANSLATION = getTranslation3d(x= Meters.of(-0.25448),y= Meters.of(0.25448),z=Meters.of(0.0508))

private val INITIAL_MODULE_3_Turn_TRANSLATION = getTranslation3d(x= Meters.of(-0.25448),y= Meters.of(-0.25448),z=Meters.of(0.0508))
private val INITIAL_MODULE_3_Drive_TRANSLATION = getTranslation3d(x= Meters.of(-0.25448),y= Meters.of(-0.25448),z=Meters.of(0.0508))

private val INITIAL_INTAKE_TRANSLATION = getTranslation3d(x = Meters.of(-0.32), z = Meters.of(0.35))
private val INITIAL_INTAKE_Roller_TRANSLATION = getTranslation3d(x = Meters.of(-0.32+0.62890), z = Meters.of(0.35))

private val INITIAL_WRIST_TRANSLATION =
    getTranslation3d(x = Meters.of(0.27434), z = Meters.of(0.79707))

private val INITIAL_ALGEA_REMOVER_ROLLER_TRANSLATION =
    getTranslation3d(x = Meters.of(0.113), z = Meters.of(0.995))

private val INITIAL_Elevator_1_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.14345040))

private val INITIAL_Elevator_2_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.20545040))

private val INITIAL_CLIMBER_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.24), z = Meters.of(0.27))



class Visualizer(
    private val swerve0Turn: Angle,
    private val swerve1Turn: Angle,
    private val swerve2Turn: Angle,
    private val swerve3Turn: Angle,
    private val swerve0Drive: Angle,
    private val swerve1Drive: Angle,
    private val swerve2Drive: Angle,
    private val swerve3Drive: Angle,
    private val extenderPosition: () -> Distance,
    private val intakeRollerAngle: () -> Angle,
    private val elevatorHeight: () -> Distance,
    private val wristAngle: () -> Angle,
    private val coralRollersAngle: () -> Angle,
    private val algaeRemoverAngle: () -> Angle,
    private val climberAngle: () -> Angle,
) {
    private val swerveDrive = frc.robot.swerveDrive
    private val vision = frc.robot.vision
    private val climber = frc.robot.climber
    private val elevator = frc.robot.elevator
    private val gripper = frc.robot.gripper
    private val extender = frc.robot.extender
    private val roller = frc.robot.roller
    private val wrist = frc.robot.wrist
    private fun getElevatorPoses(): Pair<Pose3d, Pose3d> {
        val secondStageHeight = elevator.height.invoke().`in`(Meters)
        val firstStageHeight = secondStageHeight / 2.0

        val firstStagePose = getPose3d(x=INITIAL_Elevator_1_TRANSLATION.x,z = firstStageHeight.plus(INITIAL_Elevator_1_TRANSLATION.z).minus(0.01))
        val secondStagePose = getPose3d(x=INITIAL_Elevator_2_TRANSLATION.x,z = secondStageHeight.plus(INITIAL_Elevator_2_TRANSLATION.z).minus(0.02))
        return Pair(firstStagePose, secondStagePose)
    }

    @AutoLogOutput
    fun getSubsystemsPoses(): Array<Pose3d> {
        val Module0PoseTurn = getPose3d(
            translation = INITIAL_MODULE_0_Turn_TRANSLATION,
            rotation = getRotation3d(yaw = swerveDrive.SwerveAngle[0])
        )
        val Module0PoseDrive = getPose3d(
            translation = INITIAL_MODULE_0_Drive_TRANSLATION,
            rotation = getRotation3d(yaw = swerveDrive.SwerveAngle[0])
        )

        val Module1PoseTurn = getPose3d(
            translation = INITIAL_MODULE_1_Turn_TRANSLATION,
            rotation = getRotation3d(yaw = swerveDrive.SwerveAngle[1])
        )
        val Module1PoseDrive = getPose3d(
            translation = INITIAL_MODULE_1_Drive_TRANSLATION,
            rotation = getRotation3d(yaw = swerveDrive.SwerveAngle[1])
        )
        
        val Module2PoseTurn = getPose3d(
            translation = INITIAL_MODULE_2_Turn_TRANSLATION,
            rotation = getRotation3d(yaw = swerveDrive.SwerveAngle[2])
        )
        val Module2PoseDrive = getPose3d(
            translation = INITIAL_MODULE_2_Drive_TRANSLATION,
            rotation = getRotation3d(yaw = swerveDrive.SwerveAngle[2])
        )
        
        val Module3PoseTurn = getPose3d(
            translation = INITIAL_MODULE_3_Turn_TRANSLATION,
            rotation = getRotation3d(yaw = swerveDrive.SwerveAngle[3])
        )
        val Module3PoseDrive = getPose3d(
            translation = INITIAL_MODULE_3_Drive_TRANSLATION,
            rotation = getRotation3d(yaw = swerveDrive.SwerveAngle[3])
        )
        
        val intakePose =
            getPose3d(
                INITIAL_INTAKE_TRANSLATION +
                    (getTranslation3d(x = extenderPosition.invoke()))
            )
        val intakeRollerPose = getPose3d(INITIAL_INTAKE_Roller_TRANSLATION + getTranslation3d(x=extenderPosition.invoke()))


        val (firstStagePose, secondStagePose) = getElevatorPoses()
        val wristPose = getPose3d(getTranslation3d(x=INITIAL_WRIST_TRANSLATION.x,y=INITIAL_WRIST_TRANSLATION.y,z=INITIAL_WRIST_TRANSLATION.z.plus(elevator.height.invoke().`in`(Meters))) , getRotation3d(pitch = -wristAngle.invoke()))

        val coralRollersPoseDown =wristPose
        val coralRollersPoseUp =wristPose


        val algaeRemoverPose = getPose3d(x=INITIAL_ALGEA_REMOVER_ROLLER_TRANSLATION.x,z=INITIAL_ALGEA_REMOVER_ROLLER_TRANSLATION.z.plus(elevator.height.invoke().`in`(Meters)))

        val climberPose =
            getPose3d(
                translation = INITIAL_CLIMBER_TRANSLATION,
                rotation = getRotation3d(pitch = climberAngle.invoke())
            )

        return arrayOf(
            Module0PoseTurn,
            Module0PoseDrive,
            Module1PoseTurn,
            Module1PoseDrive,
            Module2PoseTurn,
            Module2PoseDrive,
            Module3PoseTurn,
            Module3PoseDrive,
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
