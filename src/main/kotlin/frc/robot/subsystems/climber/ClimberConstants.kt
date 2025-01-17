package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia

const val UNFOLD_POWER = 1
const val FOLD_POWER = 1
const val OPEN_LATCH_POSITION = 0.8
const val CLOSE_LATCH_POSITION = 0.2
val UNFOLDED_ANGLE: Angle = Units.Degree.of(60.0)
val FOLDED_ANGLE: Angle = Units.Degree.of(30.0)
const val GEAR_RATIO = 1.0
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.0)
var DISTANCE_THRESHOLD  = Units.Centimeter.of(0.4)
var LATCH_TOLERANCE = 0.03