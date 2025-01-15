package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance

const val UNFOLD_POWER = 1
const val FOLD_POWER = 1
val OPEN_LATCH_POSITION:Distance = Units.Centimeter.of(0.8)
val  CLOSE_LATCH_POSITION:Distance = Units.Centimeter.of(0.2)
val UNFOLDED_ANGLE:Angle = Units.Degree.of(60.0)
val FOLDED_ANGLE:Angle = Units.Degree.of(30.0)
const val gearRation = 1.0