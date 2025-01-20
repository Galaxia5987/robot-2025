package frc.robot.subsystems.climber

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage

const val GEAR_RATIO = 1.0
const val STOPPER_GEAR_RATIO = 1.0
val UNLOCK_VOLTAGE: Voltage = Units.Volts.of(0.0)
val OPEN_LATCH_POSITION: Distance = Units.Millimeters.of(0.8)
val CLOSE_LATCH_POSITION: Distance = Units.Millimeters.of(0.2)
val LATCH_TOLERANCE: Distance = Units.Millimeters.of(1.0)
val UNLOCK_ANGLE: Angle = Units.Degree.of(90.0)
val LOCK_ANGLE: Angle = Units.Degree.of(10.0)
val UNFOLDED_ANGLE: Angle = Units.Degree.of(60.0)
val FOLDED_ANGLE: Angle = Units.Degree.of(30.0)
val FOLDED_TOLERANCE: Angle = Units.Degree.of(1.0)
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.0)
val MOMENT_OF_INERTIA_LOCK: MomentOfInertia = Units.KilogramSquareMeters.of(0.0)
val DISTANCE_THRESHOLD: Distance = Units.Centimeter.of(0.4)
