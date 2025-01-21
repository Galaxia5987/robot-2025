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

private val INITIAL_INTAKE_TRANSLATION = getTranslation3d(x = Meters.of(0.47), z = Meters.of(0.35))

private val INITIAL_WRIST_TRANSLATION = getTranslation3d(z = Meters.of(0.59))

private val INITIAL_CLIMBER_TRANSLATION = getTranslation3d(x = Meters.of(-0.24), z = Meters.of(0.27))

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
                INITIAL_INTAKE_TRANSLATION + (
                    getTranslation3d(x = extenderPosition.invoke())
                )
            )
        val intakeRollerPose =
            intakePose.rotateBy(
                getRotation3d(pitch = intakeRollerAngle.invoke())
            )

        val (firstStagePose, secondStagePose) = getElevatorPoses()
        val wristPose =
            secondStagePose + (
                Transform3d(
                    INITIAL_WRIST_TRANSLATION,
                    getRotation3d(pitch = -wristAngle.invoke())
                )
            )

        val coralRollersPose =
            wristPose.rotateBy(
                getRotation3d(pitch = coralRollersAngle.invoke())
            )
        val algaeRemoverPose =
            wristPose.rotateBy(
                getRotation3d(pitch = algaeRemoverAngle.invoke())
            )

        val climberPose =
            getPose3d(
                translation = INITIAL_CLIMBER_TRANSLATION,
                rotation =
                    getRotation3d(pitch = climberAngle.invoke())
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
