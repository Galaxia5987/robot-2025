package frc.robot.autonomous

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.getPose2d
import frc.robot.subsystems.drive.TunerConstants
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import kotlin.math.abs

private val xKP = LoggedNetworkNumber("/Tuning/AutoAlign/xKP", 7.0)
private val xKI = LoggedNetworkNumber("/Tuning/AutoAlign/xKI", 0.0)
private val xKD = LoggedNetworkNumber("/Tuning/AutoAlign/xKD", 0.0)
private val yKP = LoggedNetworkNumber("/Tuning/AutoAlign/yKP", 7.0)
private val yKI = LoggedNetworkNumber("/Tuning/AutoAlign/yKI", 0.0)
private val yKD = LoggedNetworkNumber("/Tuning/AutoAlign/yKD", 0.8)
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
        .apply { setTolerance(X_ALIGNMENT_TOLERANCE.`in`(Units.Meters)) }
val yController =
    ProfiledPIDController(yKP.get(), yKI.get(), yKD.get(), LINEAR_CONSTRAINTS)
        .apply { setTolerance(Y_ALIGNMENT_TOLERANCE.`in`(Units.Meters)) }
val thetaController =
    ProfiledPIDController(
        thetaKP.get(),
        thetaKI.get(),
        thetaKD.get(),
        ROTATIONAL_CONSTRAINTS
    )
        .apply {
            setTolerance(ROTATIONAL_ALIGNMENT_TOLERANCE.`in`(Units.Radians))
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
        .and(thetaController::atGoal)
        .debounce(0.15)

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
            xController.calculate(botPose.x),
            yController.calculate(botPose.y),
            thetaController.calculate(botPose.rotation.radians)
        )

    if (abs(xController.positionError) < X_ALIGNMENT_TOLERANCE.div(2.0).`in`(Units.Meters)) {
        fieldRelativeSpeeds.vxMetersPerSecond = 0.0
    }

    if (abs(yController.positionError) < Y_ALIGNMENT_TOLERANCE.div(2.0).`in`(Units.Meters)) {
        fieldRelativeSpeeds.vyMetersPerSecond = 0.0
    }

    if (abs(thetaController.positionError) < ROTATIONAL_ALIGNMENT_TOLERANCE.div(2.0).`in`(Units.Radians)) {
        fieldRelativeSpeeds.omegaRadiansPerSecond = 0.0
    }

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

    mapOf(
        "AtGoal" to atGoal.asBoolean,
        "XAtSetpoint" to xController.atSetpoint(),
        "YAtSetpoint" to yController.atSetpoint(),
        "thetaAtSetpoint" to thetaController.atSetpoint()
    )
        .forEach { (key, value) ->
            Logger.recordOutput("AutoAlignment/$key", value)
        }

    Logger.recordOutput("AutoAlignment/GoalPose", Pose2d(xController.goal.position, yController.goal.position, Rotation2d.fromRadians(thetaController.goal.position)))

    return { robotRelativeSpeeds }
}
