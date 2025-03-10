package frc.robot.subsystems.gripper

import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts

const val GEAR_RATIO = 1.0

val DISTANCE_THRESHOLD: Distance = 14.2.cm
val INTAKE_VOLTAGE: Voltage = 7.5.volts
val OUTTAKE_VOLTAGE: Voltage = 9.0.volts
val SLOW_OUTTAKE_VOLTAGE: Voltage = 2.5.volts
val FAST_OUTTAKE_VOLTAGE = 10.0.volts
val REMOVE_ALGAE_VOLTAGE: Voltage = 12.0.volts
val STOP_VOLTAGE: Voltage = 0.0.volts
val MOMENT_OF_INERTIA: MomentOfInertia = 0.03.kg2m
val DEBOUNCE_TIME: Time = 0.2.sec
