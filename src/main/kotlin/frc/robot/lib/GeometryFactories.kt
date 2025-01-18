package frc.robot.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance

fun getTranslation2d(x: Double = 0.0, y: Double = 0.0) = Translation2d(x, y)

fun getTranslation2d(x: Distance = Meters.zero(), y: Distance = Meters.zero()) = Translation2d(x, y)


fun getRotation2d(x: Double = 0.0, y: Double = 0.0) = Rotation2d(x, y)

fun getRotation2d(x: Distance = Meters.zero(), y: Distance = Meters.zero()) = Rotation2d(x.`in`(Meters), y.`in`(Meters))


fun getPose2d(translation: Translation2d = Translation2d(), rotation: Rotation2d = Rotation2d()) =
    Pose2d(translation, rotation)

fun getPose2d(x: Double = 0.0, y: Double = 0.0, rotation: Rotation2d = Rotation2d()) = Pose2d(x, y, rotation)

fun getPose2d(x: Distance = Meters.zero(), y: Distance = Meters.zero(), rotation: Rotation2d = Rotation2d()) =
    Pose2d(x, y, rotation)


fun getTranslation3d(x: Double = 0.0, y: Double = 0.0, z: Double = 0.0) = Translation3d(x, y, z)

fun getTranslation3d(x: Distance = Meters.zero(), y: Distance = Meters.zero(), z: Distance = Meters.zero()) =
    Translation3d(x, y, z)
