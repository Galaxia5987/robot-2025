package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia

val UNLOCK_VOLTAGE = Units.Volts.of(0.0)
val OPEN_LATCH_POSITION: Angle = Units.Degree.of(0.8)
val CLOSE_LATCH_POSITION:Angle = Units.Degree.of(0.2)
val UNFOLDED_ANGLE: Angle = Units.Degree.of(60.0)
val FOLDED_ANGLE: Angle = Units.Degree.of(30.0)
val FOLDED_TOLERANCE = Units.Degree.of(1.0)
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.0)
val DISTANCE_THRESHOLD = Units.Centimeter.of(0.4)
val LATCH_TOLERANCE: Angle = Units.Degree.of(1.0)
