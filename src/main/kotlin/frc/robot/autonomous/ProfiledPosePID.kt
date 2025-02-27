package frc.robot.autonomous

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.subsystems.drive.TunerConstants
import org.littletonrobotics.junction.AutoLogOutput

private val LINEAR_CONSTRAINTS = Constraints(TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond), TunerConstants.kMaxAcceleration.`in`(Units.MetersPerSecondPerSecond))

@AutoLogOutput(key = "AutoAlignment/xError")
private val xError: Distance = Units.Meters.zero()
@AutoLogOutput(key = "AutoAlignment/yError")
private val yError: Distance = Units.Meters.zero()
@AutoLogOutput(key = "AutoAlignment/thetaError")
private val thetaError: Angle = Units.Degrees.zero()

private val xTolerance: Distance = Units.Meters.zero()
private val yTolerance: Distance = Units.Meters.zero()
private val thetaTolerance: Angle = Units.Degrees.zero()

private val xController = ProfiledPIDController(0.0, 0.0, 0.0, LINEAR_CONSTRAINTS)
private val yController = ProfiledPIDController(0.0, 0.0, 0.0, LINEAR_CONSTRAINTS)
private val thetaController = PIDController(0.0, 0.0, 0.0)


