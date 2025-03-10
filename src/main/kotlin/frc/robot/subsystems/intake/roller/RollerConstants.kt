package frc.robot.subsystems.intake.roller

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.*
import frc.robot.lib.extensions.*
import frc.robot.lib.getTranslation2d

val INTAKE_VOLTAGE: Voltage = (-0.7 * 12.0).volts
val OUTTAKE_VOLTAGE: Voltage = (0.8 * 12.0).volts
val CORAL_OUTTAKE_TRANSLATION: Translation2d =
    getTranslation2d(0.3.m)
val CORAL_OUTTAKE_HEIGHT: Distance = 0.4.m
val CORAL_OUTTAKE_VELOCITY: LinearVelocity = 3.mps
val CORAL_OUTTAKE_ANGLE: Angle = 0.deg
val INTAKE_WIDTH: Distance = 0.64.m
val INTAKE_LENGTH_EXTENDED: Distance = 0.35.m
val FAR_OUTTAKE_VOLTAGE: Voltage = (0.7 * 12.0).volts
const val GEAR_RATIO = 1.0
val MOMENT_OF_INERTIA: MomentOfInertia = 0.003.kg2m
