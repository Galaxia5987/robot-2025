package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val MAX_HEIGHT = 1.3
const val GEAR_RATIO = (1.0 / 12.0) * (42.0 / 48.0)
const val FIRST_STAGE_RATIO = 2.0
val SPROCKET_RADIUS: Distance = Units.Millimeters.of(36.4/2.0)
val GAINS = selectGainsBasedOnMode(
    Gains(
        20.0,
        kD=1.0
    ),
    Gains())
