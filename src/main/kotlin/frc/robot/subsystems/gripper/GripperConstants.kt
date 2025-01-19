package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.MomentOfInertia

const val INTAKE_POWER = 0.3 // TODO: Calibrate
const val OUTTAKE_POWER = -0.3 // TODO: Calibrate
const val REMOVE_ALGAE_POWER = 0.3 // TODO: Calibrate
const val GEAR_RATIO = 1.0 // TODO: Replace with real value
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.03) // TODO: Replace with real value
