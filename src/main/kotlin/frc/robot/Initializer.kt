package frc.robot

import frc.robot.generated.TunerConstants
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.GyroIO
import frc.robot.subsystems.drive.GyroIONavX
import frc.robot.subsystems.drive.ModuleIO
import frc.robot.subsystems.drive.ModuleIOSim
import frc.robot.subsystems.drive.ModuleIOTalonFX
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIO
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim

private var swerveDrive: Drive? = null
private var vision: Vision? = null

private fun getSwerveModuleIOs(): Array<ModuleIO> {
    return when (CURRENT_MODE) {
        Mode.REAL -> arrayOf(
            ModuleIOTalonFX(TunerConstants.FrontLeft),
            ModuleIOTalonFX(TunerConstants.FrontRight),
            ModuleIOTalonFX(TunerConstants.BackLeft),
            ModuleIOTalonFX(TunerConstants.BackRight)
        )
        Mode.SIM -> arrayOf(
            ModuleIOSim(TunerConstants.FrontLeft),
            ModuleIOSim(TunerConstants.FrontRight),
            ModuleIOSim(TunerConstants.BackLeft),
            ModuleIOSim(TunerConstants.BackRight)
        )
        Mode.REPLAY -> arrayOf(
            object : ModuleIO {},
            object : ModuleIO {},
            object : ModuleIO {},
            object : ModuleIO {}
        )
    }
}

private fun getGyroIO(): GyroIO = when (CURRENT_MODE) {
    Mode.REAL -> GyroIONavX()
    Mode.SIM -> object : GyroIO {}
    Mode.REPLAY -> object : GyroIO {}
}

private fun initSwerve() {
    if (swerveDrive == null) {
        swerveDrive = Drive(getGyroIO(), getSwerveModuleIOs())
    }
}

fun getSwerve(): Drive {
    return swerveDrive ?: throw IllegalStateException("Swerve has not been initialized.")
}

private fun getVisionIOs(): Array<VisionIO> = when (CURRENT_MODE) {
    Mode.REAL -> VisionConstants.OVNameToTransform.map { VisionIOPhotonVision(it.key, it.value) }.toTypedArray()
    Mode.SIM -> VisionConstants.OVNameToTransform.map { VisionIOPhotonVisionSim(it.key, it.value, getSwerve()::getPose) }.toTypedArray()
    Mode.REPLAY -> emptyArray()
}

fun initVision() {
    vision = Vision(getSwerve()::addVisionMeasurement, *getVisionIOs())
}

fun getVision(): Vision {
    return vision ?: throw IllegalStateException("Vision has npt been initialized.")
}

fun initializeSubsystems() {
    initSwerve()
    initVision()
}
