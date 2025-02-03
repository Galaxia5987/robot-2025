package frc.robot.subsystems.intake.roller

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage

val INTAKE_VOLTAGE: Voltage = Units.Volts.of(-0.7 * 12.0)
val OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(0.8 * 12.0)
val FAR_OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(0.7 * 12.0)
const val GEAR_RATIO = 1.0
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.003)
