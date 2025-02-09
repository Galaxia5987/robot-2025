package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage

const val GEAR_RATIO = 1.0 // TODO: Replace with real value

val DISTANCE_THRESHOLD: Distance = Units.Centimeters.of(14.2)
val INTAKE_VOLTAGE: Voltage = Units.Volts.of(7.5)
val OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(-9.0)
val SLOW_OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(-1.5)
val REMOVE_ALGAE_VOLTAGE: Voltage = Units.Volts.of(12.0) // TODO: Calibrate
val STOP_VOLTAGE: Voltage = Units.Volts.zero()
val MOMENT_OF_INERTIA: MomentOfInertia =
    Units.KilogramSquareMeters.of(0.03) // TODO: Replace with real value
val DEBOUNCE_TIME: Time = Units.Seconds.of(0.2)
