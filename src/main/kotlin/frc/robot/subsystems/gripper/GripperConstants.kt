package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage

const val GEAR_RATIO = 1.0

val DISTANCE_THRESHOLD: Distance = Units.Centimeters.of(14.2)
val INTAKE_VOLTAGE: Voltage = Units.Volts.of(7.5)
val OUTTAKE_L3: Voltage = Units.Volts.of(4.0)
val OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(12.0)
val SLOW_OUTTAKE_VOLTAGE: Voltage = Units.Volts.of(3.5)
val OUTTAKE_L1_VOLTAGE: Voltage = Units.Volts.of(-4.3)
val FAST_OUTTAKE_VOLTAGE = Units.Volts.of(10.0)
val REMOVE_ALGAE_VOLTAGE: Voltage = Units.Volts.of(12.0)
val INTAKE_ALGAE_VOLTAGE: Voltage = Units.Volts.of(-12.0)
val OUTTAKE_ALGAE_VOLTAGE: Voltage = Units.Volts.of(10.0)
val STOP_VOLTAGE: Voltage = Units.Volts.zero()
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.03)
val DEBOUNCE_TIME: Time = Units.Seconds.of(0.2)
val CLOSE_WRIST_DEBOUNCE_TIME: Time = Units.Seconds.of(2.0)
val ALGAE_CURRENT: Current = Units.Amps.of(55.0)
val STATOR_CURRENT_LIMIT: Current = Units.Amps.of(80.0)
