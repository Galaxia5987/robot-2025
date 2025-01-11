package frc.robot

import frc.robot.subsystems.drive.TunerConstants
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.GyroIO
import frc.robot.subsystems.drive.GyroIONavX
import frc.robot.subsystems.drive.ModuleIO
import frc.robot.subsystems.drive.ModuleIOSim
import frc.robot.subsystems.drive.ModuleIOTalonFX

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
