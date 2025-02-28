package frc.robot.autonomous

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.Units
import frc.robot.subsystems.drive.TunerConstants
import org.littletonrobotics.junction.Logger

private val LINEAR_CONSTRAINTS = Constraints(TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond), TunerConstants.kMaxAcceleration.`in`(Units.MetersPerSecondPerSecond))

val xController = ProfiledPIDController(1.0, 0.0, 0.0, LINEAR_CONSTRAINTS).apply {
    setTolerance(Units.Meters.of(0.03).`in`(Units.Meters))
}
val yController = ProfiledPIDController(1.0, 0.0, 0.0, LINEAR_CONSTRAINTS).apply {
    setTolerance(Units.Meters.of(0.03).`in`(Units.Meters))
}
val thetaController = PIDController(1.0, 0.0, 0.0).apply {
    setTolerance(Units.Degrees.of(2.0).`in`(Units.Radians))
    enableContinuousInput(-Math.PI, Math.PI)
}

fun setGoal(desiredPose: Pose2d) {
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setpoint = desiredPose.rotation.radians
}

fun atGoal(): () -> Boolean = { xController.atGoal() && yController.atGoal() && thetaController.atSetpoint() }

fun getSpeed(botPose: Pose2d): () -> ChassisSpeeds {
    val fieldRelativeSpeeds = ChassisSpeeds(
        xController.calculate(botPose.x),
        yController.calculate(botPose.y),
        thetaController.calculate(botPose.rotation.radians),
    )
    val robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, botPose.rotation)

    mapOf(
        "XError" to xController.positionError,
        "YError" to yController.positionError,
        "ThetaError" to thetaController.error,
    ).forEach { (key, value) -> Logger.recordOutput("AutoAlignment/$key", value) }

    mapOf(
        "XGoal" to xController.goal.position,
        "YGoal" to yController.goal.position,
        "ThetaGoal" to thetaController.setpoint,
    ).forEach { (key, value) -> Logger.recordOutput("AutoAlignment/$key", value) }

    Logger.recordOutput("AutoAlignment/AtGoal", atGoal())
    Logger.recordOutput("AutoAlignment/AtGoal", atGoal())
    Logger.recordOutput("AutoAlignment/AtGoal", atGoal())

    return { robotRelativeSpeeds }
}


