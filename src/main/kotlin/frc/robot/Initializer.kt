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

fun getSwerve(): Drive {
    return Drive(getGyroIO(), getSwerveModuleIOs())
}

private fun getVisionIOs(): Array<VisionIO> {
    return when(CURRENT_MODE){
        Mode.REAL -> arrayOf(
            VisionIOPhotonVision(VisionConstants.FrontOVName, VisionConstants.robotToFrontOV),
            VisionIOPhotonVision(VisionConstants.LeftOVName, VisionConstants.robotToLeftOV),
            VisionIOPhotonVision(VisionConstants.RightOVName, VisionConstants.robotToRightOV)
        )
        Mode.SIM -> arrayOf(
            VisionIOPhotonVisionSim(VisionConstants.FrontOVName, VisionConstants.robotToFrontOV, getSwerve()::getPose),
            VisionIOPhotonVisionSim(VisionConstants.LeftOVName, VisionConstants.robotToLeftOV, getSwerve()::getPose),
            VisionIOPhotonVisionSim(VisionConstants.RightOVName, VisionConstants.robotToRightOV, getSwerve()::getPose)
        )
        Mode.REPLAY -> arrayOf()
    }
}

fun getVision(): Vision {
    return Vision(getSwerve()::addVisionMeasurement, *getVisionIOs())
}
