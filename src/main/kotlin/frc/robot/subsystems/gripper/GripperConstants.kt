package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage

const val GEAR_RATIO = 1.0 // TODO: Replace with real value

val INTAKE_VOLTAGE: Voltage = Units.Volts.of(0.3) // TODO: Calibrate
val OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(-0.3) // TODO: Calibrate
val REMOVE_ALGAE_VOLTAGE: Voltage = Units.Volts.of(0.3) // TODO: Calibrate
val STOP_VOLTAGE = Units.Volts.zero()
val MOMENT_OF_INERTIA: MomentOfInertia =
    Units.KilogramSquareMeters.of(0.03) // TODO: Replace with real value
