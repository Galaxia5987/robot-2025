package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import org.littletonrobotics.junction.AutoLogOutput

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

        val firstStagePose =
            Pose3d(0.0, 0.0, firstStageHeight, Rotation3d.kZero)
        val secondStagePose =
            Pose3d(0.0, 0.0, secondStageHeight, Rotation3d.kZero)
        return Pair(firstStagePose, secondStagePose)
    }

    @AutoLogOutput
    fun visualizeSubsystems(): Array<Pose3d> {
        val intakePose =
            Pose3d(
                extenderPosition.invoke().`in`(Meters),
                0.0,
                0.0,
                Rotation3d.kZero
            )
        val intakeRollerPose =
            intakePose.rotateBy(
                Rotation3d(0.0, intakeRollerAngle.invoke().`in`(Radians), 0.0)
            )

        val (firstStagePose, secondStagePose) = getElevatorPoses()
        val wristPose =
            secondStagePose.rotateBy(
                Rotation3d(0.0, wristAngle.invoke().`in`(Radians), 0.0)
            )

        val coralRollersPose =
            wristPose.rotateBy(
                Rotation3d(0.0, coralRollersAngle.invoke().`in`(Radians), 0.0)
            )
        val algaeRemoverPose =
            wristPose.rotateBy(
                Rotation3d(0.0, algaeRemoverAngle.invoke().`in`(Radians), 0.0)
            )

        val climberPose =
            Pose3d(
                0.0,
                0.0,
                0.0,
                Rotation3d(0.0, climberAngle.invoke().`in`(Radians), 0.0)
            )

        return arrayOf(
            intakePose,
            intakeRollerPose,
            firstStagePose,
            secondStagePose,
            // The two identical poses are for the upper and lower rollers.
            wristPose,
            coralRollersPose,
            coralRollersPose,
            algaeRemoverPose,
            climberPose
        )
    }
}
