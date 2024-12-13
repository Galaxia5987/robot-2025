package frc.robot.subsystems.drive

import frc.robot.Constants
import frc.robot.generated.TunerConstants

fun getSwerveModuleIOs(): Array<ModuleIO> {
    return when (Constants.CURRENT_MODE) {
        Constants.Mode.REAL -> arrayOf(
            ModuleIOTalonFX(TunerConstants.FrontLeft),
            ModuleIOTalonFX(TunerConstants.FrontRight),
            ModuleIOTalonFX(TunerConstants.BackLeft),
            ModuleIOTalonFX(TunerConstants.BackRight)
        )
        Constants.Mode.SIM -> arrayOf(
            ModuleIOSim(TunerConstants.FrontLeft),
            ModuleIOSim(TunerConstants.FrontRight),
            ModuleIOSim(TunerConstants.BackLeft),
            ModuleIOSim(TunerConstants.BackRight)
        )
        Constants.Mode.REPLAY -> arrayOf(
            object : ModuleIO {},
            object : ModuleIO {},
            object : ModuleIO {},
            object : ModuleIO {}
        )
    }
}

fun getGyroIO(): GyroIO = when (Constants.CURRENT_MODE) {
    Constants.Mode.REAL -> GyroIONavX()
    Constants.Mode.SIM -> object : GyroIO {}
    Constants.Mode.REPLAY -> object : GyroIO {}
}