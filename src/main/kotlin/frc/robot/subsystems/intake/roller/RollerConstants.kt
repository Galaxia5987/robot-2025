package frc.robot.subsystems.intake.roller

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*
import frc.robot.lib.getTranslation2d

val INTAKE_VOLTAGE: Voltage = Units.Volts.of(-0.7 * 12.0)
val SLOW_INTAKE_VOLTAGE: Voltage = Units.Volts.of(-0.3 * 12.0)
val OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(0.8 * 12.0)
val SLOW_OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(0.4 * 12.0)
val CORAL_OUTTAKE_TRANSLATION: Translation2d =
    getTranslation2d(Units.Meters.of(0.3))
val CORAL_OUTTAKE_HEIGHT: Distance = Units.Meter.of(0.4)
val CORAL_OUTTAKE_VELOCITY: LinearVelocity = Units.MetersPerSecond.of(3.0)
val CORAL_OUTTAKE_ANGLE: Angle = Units.Degrees.zero()
val INTAKE_WIDTH: Distance = Units.Meters.of(0.64)
val INTAKE_LENGTH_EXTENDED: Distance = Units.Meters.of(0.35)
val FAR_OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(0.7 * 12.0)
const val GEAR_RATIO = 1.0
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.003)
