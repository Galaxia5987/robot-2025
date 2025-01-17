package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance

class Visualizer(
    private val elevatorHeight: () -> Distance,
    private val wristAngle: () -> Angle,
    private val extenderPosition: () -> Distance
) {
}