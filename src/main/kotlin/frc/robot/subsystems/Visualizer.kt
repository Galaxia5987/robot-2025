package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance

class Visualizer(
    private val elevatorHeight: () -> Distance,
    private val wristAngle: () -> Angle,
    private val extenderPosition: () -> Distance
) {
    private fun getElevatorPoses(): Pair<Pose3d, Pose3d> {
        val carriageHeight = elevatorHeight.invoke().`in`(Meters)
        val firstStageHeight = carriageHeight / 2.0

        val firstStagePose = Pose3d(0.0, 0.0, firstStageHeight, Rotation3d.kZero)
        val secondStagePose = Pose3d(0.0, 0.0, firstStageHeight, Rotation3d.kZero)
        return Pair(firstStagePose, secondStagePose)
    }
}