package frc.robot

import frc.robot.generated.TunerConstants
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim

private val swerveModuleIOs =
    when (CURRENT_MODE) {
        Mode.REAL ->
            arrayOf(
                ModuleIOTalonFX(TunerConstants.FrontLeft),
                ModuleIOTalonFX(TunerConstants.FrontRight),
                ModuleIOTalonFX(TunerConstants.BackLeft),
                ModuleIOTalonFX(TunerConstants.BackRight),
            )
        Mode.SIM ->
            arrayOf(
                ModuleIOSim(TunerConstants.FrontLeft),
                ModuleIOSim(TunerConstants.FrontRight),
                ModuleIOSim(TunerConstants.BackLeft),
                ModuleIOSim(TunerConstants.BackRight),
            )
        Mode.REPLAY -> arrayOf()
    }

private val gyroIO =
    when (CURRENT_MODE) {
        Mode.REAL -> GyroIONavX()
        else -> object : GyroIO {}
    }

val swerveDrive = Drive(gyroIO, swerveModuleIOs)

private val visionIOs =
    when (CURRENT_MODE) {
        Mode.REAL ->
            VisionConstants.OVNameToTransform.map {
                VisionIOPhotonVision(it.key, it.value)
            }
        Mode.SIM ->
            VisionConstants.OVNameToTransform.map {
                VisionIOPhotonVisionSim(it.key, it.value, swerveDrive::getPose)
            }
        Mode.REPLAY -> emptyList()
    }.toTypedArray()

val vision = Vision(swerveDrive::addVisionMeasurement, *visionIOs)
