package frc.robot.autonomous

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.Units
import frc.robot.subsystems.drive.TunerConstants

private val LINEAR_CONSTRAINTS = Constraints(TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond), TunerConstants.kMaxAcceleration.`in`(Units.MetersPerSecondPerSecond))

private val xController = ProfiledPIDController(0.0, 0.0, 0.0, LINEAR_CONSTRAINTS).apply {
    setTolerance(Units.Meters.of(0.0).`in`(Units.Meters))
}
private val yController = ProfiledPIDController(0.0, 0.0, 0.0, LINEAR_CONSTRAINTS).apply {
    setTolerance(Units.Meters.of(0.0).`in`(Units.Meters))
}
private val thetaController = PIDController(0.0, 0.0, 0.0).apply {
    setTolerance(Units.Degrees.of(0.0).`in`(Units.Radians))
}

fun setSetpoint(desiredPose: Pose2d) {
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setpoint = desiredPose.rotation.radians
}

fun getSpeed(botPose: Pose2d): ChassisSpeeds {
    val fieldRelativeSpeeds = ChassisSpeeds(
        xController.calculate(botPose.x),
        yController.calculate(botPose.y),
        thetaController.calculate(botPose.rotation.radians),
    )
    val robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, botPose.rotation)

    return robotRelativeSpeeds
}


