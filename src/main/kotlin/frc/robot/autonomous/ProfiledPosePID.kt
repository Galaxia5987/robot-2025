package frc.robot.autonomous

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
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

val xController =
    ProfiledPIDController(xKP.get(), xKI.get(), xKD.get(), LINEAR_CONSTRAINTS).apply {
        setTolerance(Units.Meters.of(0.05).`in`(Units.Meters))
    }
val yController =
    ProfiledPIDController(yKP.get(), yKI.get(), yKD.get(), LINEAR_CONSTRAINTS).apply {
        setTolerance(Units.Meters.of(0.05).`in`(Units.Meters))
    }
val thetaController =
    PIDController(thetaKP.get(), thetaKI.get(), thetaKD.get()).apply {
        setTolerance(Units.Degrees.of(2.0).`in`(Units.Radians))
        enableContinuousInput(-Math.PI, Math.PI)
    }

fun updateProfiledPID(){
    xController.setPID(xKP.get(), xKI.get(), xKD.get())
    yController.setPID(yKP.get(), yKI.get(), yKD.get())
    thetaController.setPID(thetaKP.get(), thetaKI.get(), thetaKD.get())
}

fun setGoal(desiredPose: Pose2d) {
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setpoint = desiredPose.rotation.radians
}

val atGoal =
    Trigger {
            xController.atGoal() &&
                yController.atGoal() &&
                thetaController.atSetpoint()
        }
        .debounce(0.4)

fun resetProfiledPID(botPose: Pose2d, botSpeeds: ChassisSpeeds) {
    xController.reset(botPose.x, botSpeeds.vxMetersPerSecond)
    yController.reset(botPose.y, botSpeeds.vyMetersPerSecond)
}

fun getSpeed(botPose: Pose2d): () -> ChassisSpeeds {
    val fieldRelativeSpeeds =
        ChassisSpeeds(
            MathUtil.applyDeadband(xController.calculate(botPose.x), Units.Meters.of(0.05).`in`(Units.Meters)),
            MathUtil.applyDeadband(yController.calculate(botPose.y), Units.Meters.of(0.05).`in`(Units.Meters)),
            MathUtil.applyDeadband(thetaController.calculate(botPose.rotation.radians), Units.Degrees.of(1.0).`in`(Units.Radians)),
        )
    val robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            botPose.rotation
        )

    mapOf(
            "XError" to xController.positionError,
            "YError" to yController.positionError,
            "ThetaError" to thetaController.error,
        )
        .forEach { (key, value) ->
            Logger.recordOutput("AutoAlignment/$key", value)
        }

    mapOf(
        "XSetpoint" to xController.setpoint.position,
        "YSetpoint" to yController.setpoint.position,
        "ThetaSetpoint" to thetaController.setpoint,
    )
        .forEach { (key, value) ->
            Logger.recordOutput("AutoAlignment/$key", value)
        }

    mapOf(
            "XGoal" to xController.goal.position,
            "YGoal" to yController.goal.position,
            "ThetaGoal" to thetaController.setpoint,
        )
        .forEach { (key, value) ->
            Logger.recordOutput("AutoAlignment/$key", value)
        }

    Logger.recordOutput("AutoAlignment/AtGoal", atGoal)

    return { robotRelativeSpeeds }
}
