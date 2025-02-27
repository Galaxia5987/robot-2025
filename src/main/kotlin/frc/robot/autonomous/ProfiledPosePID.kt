package frc.robot.autonomous

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.subsystems.drive.TunerConstants

private val LINEAR_CONSTRAINTS = Constraints(TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond), TunerConstants.kMaxAcceleration.`in`(Units.MetersPerSecondPerSecond))

private val xError: Distance = Units.Meters.zero()
private val yError: Distance = Units.Meters.zero()
private val thetaError: Distance = Units.Meters.zero()

private val xTolerance: Distance = Units.Meters.zero()
private val yTolerance: Distance = Units.Meters.zero()
private val thetaTolerance: Distance = Units.Meters.zero()

private val xController = ProfiledPIDController(0.0, 0.0, 0.0, LINEAR_CONSTRAINTS)
private val yController = ProfiledPIDController(0.0, 0.0, 0.0, LINEAR_CONSTRAINTS)
private val thetaController = PIDController(0.0, 0.0, 0.0)


