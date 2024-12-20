package frc.robot.subsystems.elevator

import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val MAX_HEIGHT = 1.3
const val SPROCKET_RADIUS= 12.13
const val GEAR_RATIO = (1 / 12) * (42 / 48)
const val FIRST_STAGE_RATIO = 0.5
val GAINS = selectGainsBasedOnMode(Gains(), Gains())
    Gains(
        20.0,
        kD=1.0
    ),
    Gains())
