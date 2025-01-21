package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import org.littletonrobotics.junction.AutoLogOutput

private val INTAKE_TRANSLATION = getTranslation3d(x = 0.47, z = 0.35)

private val WRIST_TRANSLATION = getTranslation3d(z = 0.59)

private val CLIMBER_TRANSLATION = getTranslation3d(x = -0.24, z = 0.27)

class Visualizer(
    private val extenderPosition: () -> Distance,
    private val intakeRollerAngle: () -> Angle,
    private val elevatorHeight: () -> Distance,
    private val wristAngle: () -> Angle,
    private val coralRollersAngle: () -> Angle,
    private val algaeRemoverAngle: () -> Angle,
    private val climberAngle: () -> Angle,
) {
    private fun getElevatorPoses(): Pair<Pose3d, Pose3d> {
        val secondStageHeight = elevatorHeight.invoke().`in`(Meters)
        val firstStageHeight = secondStageHeight / 2.0

        val firstStagePose = getPose3d(z = firstStageHeight)
        val secondStagePose = getPose3d(z = secondStageHeight)
        return Pair(firstStagePose, secondStagePose)
    }

    @AutoLogOutput
    fun visualizeSubsystems(): Array<Pose3d> {
        val intakePose =
            getPose3d(
                INTAKE_TRANSLATION.plus(
                    getTranslation3d(x = extenderPosition.invoke().`in`(Meters))
                )
            )
        val intakeRollerPose =
            intakePose.rotateBy(
                getRotation3d(pitch = intakeRollerAngle.invoke().`in`(Radians))
            )

        val (firstStagePose, secondStagePose) = getElevatorPoses()
        val wristPose =
            secondStagePose.plus(
                Transform3d(
                    WRIST_TRANSLATION,
                    getRotation3d(pitch = -wristAngle.invoke().`in`(Radians))
                )
            )

        val coralRollersPose =
            wristPose.rotateBy(
                getRotation3d(pitch = coralRollersAngle.invoke().`in`(Radians))
            )
        val algaeRemoverPose =
            wristPose.rotateBy(
                getRotation3d(pitch = algaeRemoverAngle.invoke().`in`(Radians))
            )

        val climberPose =
            getPose3d(
                translation = CLIMBER_TRANSLATION,
                rotation =
                    getRotation3d(pitch = climberAngle.invoke().`in`(Radians))
            )

        return arrayOf(
            intakePose,
            //            intakeRollerPose,
            firstStagePose,
            secondStagePose,
            wristPose,
            // The two identical poses are for the upper and lower rollers.
            //            coralRollersPose,
            //            coralRollersPose,
            //            algaeRemoverPose,
            climberPose
        )
    }
}
