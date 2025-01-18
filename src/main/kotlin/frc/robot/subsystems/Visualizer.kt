package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import org.littletonrobotics.junction.AutoLogOutput

class Visualizer(
    private val elevatorHeight: () -> Distance,
    private val wristAngle: () -> Angle,
    private val extenderPosition: () -> Distance,
    private val climberAngle: () -> Angle,
    private val coralRollersAngle: () -> Angle,
) {
    private fun getElevatorPoses(): Pair<Pose3d, Pose3d> {
        val carriageHeight = elevatorHeight.invoke().`in`(Meters)
        val firstStageHeight = carriageHeight / 2.0

        val firstStagePose = Pose3d(0.0, 0.0, firstStageHeight, Rotation3d.kZero)
        val secondStagePose = Pose3d(0.0, 0.0, firstStageHeight, Rotation3d.kZero)
        return Pair(firstStagePose, secondStagePose)
    }

    private fun getWristPose(carriagePose: Pose3d): Pose3d {
        val wristAngleDouble = wristAngle.invoke().`in`(Radians)
        return carriagePose.rotateBy(Rotation3d(0.0, wristAngleDouble, 0.0))
    }

    @AutoLogOutput
    fun visualize(): Array<Pose3d> {
        val intakePose = Pose3d(extenderPosition.invoke().`in`(Meters), 0.0, 0.0, Rotation3d.kZero)
        val (firstStagePose, secondStagePose) = getElevatorPoses()
        val wristPose = getWristPose(secondStagePose)
        val climberPose = Pose3d(0.0, 0.0, 0.0, Rotation3d(0.0, climberAngle.invoke().`in`(Radians), 0.0))

        val coralRollersPose = wristPose.rotateBy(Rotation3d(0.0, coralRollersAngle.invoke().`in`(Radians), 0.0))

        return arrayOf(
            intakePose,
            firstStagePose,
            secondStagePose,
            wristPose,
            coralRollersPose,  // Upper roller.
            coralRollersPose,  // Lower roller.
            climberPose
        )
    }
}