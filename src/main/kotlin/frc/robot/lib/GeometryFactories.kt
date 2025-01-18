package frc.robot.lib

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance

fun getTranslation2d(x: Double = 0.0, y: Double = 0.0) = Translation2d(x, y)

fun getTranslation2d(x: Distance = Meters.zero(), y: Distance = Meters.zero()) = Translation2d(x, y)
