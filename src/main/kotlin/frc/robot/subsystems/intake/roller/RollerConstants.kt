package frc.robot.subsystems.intake.roller

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.MomentOfInertia


val INTAKE_WIDTH = Units.Meters.of(0.64)
val INTAKE_LENGTH_EXTENDED = Units.Meters.of(0.35)
val INTAKE_VOLTAGE = Units.Volts.of(0.4 * 12.0)
val OUTTAKE_VOLTAGE = Units.Volts.of(0.5 * 12.0)
val FAR_OUTTAKE_VOLTAGE = Units.Volts.of(0.7 * 12.0)
const val GEAR_RATIO = 1.0
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.003)
