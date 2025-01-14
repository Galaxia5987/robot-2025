package frc.robot

import frc.robot.subsystems.drive.*
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim

private val swerveModuleIOs =
    arrayOf(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        )
        .map { module ->
            when (CURRENT_MODE) {
                Mode.REAL -> ModuleIOTalonFX(module)
                Mode.SIM -> ModuleIOSim(module)
                Mode.REPLAY -> object : ModuleIO {}
            }
        }
        .toTypedArray()

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
