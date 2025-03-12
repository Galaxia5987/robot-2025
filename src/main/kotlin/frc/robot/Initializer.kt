package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.drive.*
import java.lang.Exception
import java.util.*
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation

val driveSimulation: SwerveDriveSimulation? =
    if (CURRENT_MODE == Mode.SIM && USE_MAPLE_SIM)
        SwerveDriveSimulation(
                Drive.mapleSimConfig,
                Pose2d(3.0, 3.0, Rotation2d())
            )
            .apply {
                SimulatedArena.getInstance().addDriveTrainSimulation(this)
            }
    else null

private val swerveModuleIOs =
    arrayOf(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        )
        .mapIndexed { index, module ->
            when (CURRENT_MODE) {
                Mode.REAL -> ModuleIOTalonFX(module)
                Mode.SIM ->
                    if (USE_MAPLE_SIM)
                        ModuleIOMapleSim(
                            driveSimulation?.modules?.get(index)
                                ?: throw Exception("Sim Swerve Module is null")
                        )
                    else ModuleIOSim(module)
                Mode.REPLAY -> object : ModuleIO {}
            }
        }
        .toTypedArray()

private val gyroIO =
    when (CURRENT_MODE) {
        Mode.REAL -> GyroIONavX()
        Mode.SIM ->
            if (USE_MAPLE_SIM)
                GyroIOSim(
                    driveSimulation?.gyroSimulation
                        ?: throw Exception("Gyro simulation is null")
                )
            else object : GyroIO {}
        else -> object : GyroIO {}
    }

val swerveDrive =
    Drive(gyroIO, swerveModuleIOs, Optional.ofNullable(driveSimulation))
