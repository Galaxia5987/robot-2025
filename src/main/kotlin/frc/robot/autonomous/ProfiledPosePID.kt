package frc.robot.autonomous

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.drive.TunerConstants
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

private val xKP = LoggedNetworkNumber("/Tuning/AutoAlign/xKP", 5.0)
private val xKI = LoggedNetworkNumber("/Tuning/AutoAlign/xKI", 0.0)
private val xKD = LoggedNetworkNumber("/Tuning/AutoAlign/xKD", 0.0)
private val yKP = LoggedNetworkNumber("/Tuning/AutoAlign/yKP", 5.0)
private val yKI = LoggedNetworkNumber("/Tuning/AutoAlign/yKI", 0.0)
private val yKD = LoggedNetworkNumber("/Tuning/AutoAlign/yKD", 0.0)
private val thetaKP = LoggedNetworkNumber("/Tuning/AutoAlign/thetaKP", 6.0)
private val thetaKI = LoggedNetworkNumber("/Tuning/AutoAlign/thetaKI", 0.0)
private val thetaKD = LoggedNetworkNumber("/Tuning/AutoAlign/thetaKD", 0.0)

private val LINEAR_CONSTRAINTS =
    Constraints(
        TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond),
        TunerConstants.kMaxAcceleration.`in`(Units.MetersPerSecondPerSecond)
    )

private val ROTATIONAL_CONSTRAINTS =
    Constraints(
        TunerConstants.kMaxOmegaVelocity.`in`(Units.RadiansPerSecond),
        TunerConstants.kMaxAngularAcceleration.`in`(
            Units.RadiansPerSecondPerSecond
        )
    )

val xController =
    ProfiledPIDController(xKP.get(), xKI.get(), xKD.get(), LINEAR_CONSTRAINTS)
        .apply { setTolerance(Units.Meters.of(0.02).`in`(Units.Meters)) }
val yController =
    ProfiledPIDController(yKP.get(), yKI.get(), yKD.get(), LINEAR_CONSTRAINTS)
        .apply { setTolerance(Units.Meters.of(0.02).`in`(Units.Meters)) }
val thetaController =
    ProfiledPIDController(
            thetaKP.get(),
            thetaKI.get(),
            thetaKD.get(),
            ROTATIONAL_CONSTRAINTS
        )
        .apply {
            setTolerance(Units.Degrees.of(1.0).`in`(Units.Radians))
            enableContinuousInput(-Math.PI, Math.PI)
        }

fun updateProfiledPID() {
    xController.setPID(xKP.get(), xKI.get(), xKD.get())
    yController.setPID(yKP.get(), yKI.get(), yKD.get())
    thetaController.setPID(thetaKP.get(), thetaKI.get(), thetaKD.get())
}

fun setGoal(desiredPose: Pose2d) {
    updateProfiledPID()
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setGoal(desiredPose.rotation.radians)
}

val atGoal: Trigger =
    Trigger(xController::atGoal)
        .and(yController::atGoal)
        .and(thetaController::atSetpoint)
        .debounce(0.4)

fun resetProfiledPID(botPose: Pose2d, botSpeeds: ChassisSpeeds) {
    xController.reset(botPose.x, botSpeeds.vxMetersPerSecond)
    yController.reset(botPose.y, botSpeeds.vyMetersPerSecond)
    thetaController.reset(
        botPose.rotation.radians,
        botSpeeds.omegaRadiansPerSecond
    )
}

fun getSpeed(botPose: Pose2d): () -> ChassisSpeeds {
    val fieldRelativeSpeeds =
        ChassisSpeeds(
            MathUtil.applyDeadband(
                xController.calculate(botPose.x),
                Units.Meters.of(0.02).`in`(Units.Meters)
            ),
            MathUtil.applyDeadband(
                yController.calculate(botPose.y),
                Units.Meters.of(0.02).`in`(Units.Meters)
            ),
            MathUtil.applyDeadband(
                thetaController.calculate(botPose.rotation.radians),
                Units.Degrees.of(1.0).`in`(Units.Radians)
            ),
        )
    val robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            botPose.rotation
        )

    mapOf(
            "XError" to xController.positionError,
            "YError" to yController.positionError,
            "ThetaError" to thetaController.positionError,
            "XSetpoint" to xController.setpoint.position,
            "YSetpoint" to yController.setpoint.position,
            "ThetaSetpoint" to thetaController.setpoint.position,
            "XGoal" to xController.goal.position,
            "YGoal" to yController.goal.position,
            "ThetaGoal" to thetaController.goal.position,
        )
        .forEach { (key, value) ->
            Logger.recordOutput("AutoAlignment/$key", value)
        }

    Logger.recordOutput("AutoAlignment/AtGoal", atGoal)

    return { robotRelativeSpeeds }
}
