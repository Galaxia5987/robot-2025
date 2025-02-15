package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.climber.ClimberIO
import frc.robot.subsystems.climber.ClimberIOReal
import frc.robot.subsystems.climber.ClimberIOSim
import frc.robot.subsystems.climber.LoggedClimberInputs
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorIO
import frc.robot.subsystems.elevator.ElevatorIOReal
import frc.robot.subsystems.elevator.ElevatorIOSim
import frc.robot.subsystems.elevator.LoggedElevatorInputs
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.gripper.GripperIO
import frc.robot.subsystems.gripper.GripperIOReal
import frc.robot.subsystems.gripper.GripperIOSim
import frc.robot.subsystems.gripper.LoggedGripperInputs
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderIO
import frc.robot.subsystems.intake.extender.ExtenderIOReal
import frc.robot.subsystems.intake.extender.ExtenderIOSim
import frc.robot.subsystems.intake.extender.LoggedExtenderInputs
import frc.robot.subsystems.intake.roller.*
import frc.robot.subsystems.leds.LEDs
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim
import frc.robot.subsystems.wrist.LoggedWristInputs
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristIO
import frc.robot.subsystems.wrist.WristIOReal
import frc.robot.subsystems.wrist.WristIOSim
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

private val visionIOs =
    when (CURRENT_MODE) {
        Mode.REAL ->
            VisionConstants.OVNameToTransform.map {
                VisionIOPhotonVision(it.key, it.value)
            }
        Mode.SIM ->
            VisionConstants.OVNameToTransform.map {
                VisionIOPhotonVisionSim(
                    it.key,
                    it.value,
                    if (USE_MAPLE_SIM)
                        driveSimulation!!::getSimulatedDriveTrainPose
                    else swerveDrive::getPose
                )
            }
        Mode.REPLAY -> emptyList()
    }.toTypedArray()

val vision = Vision(swerveDrive::addVisionMeasurement, *visionIOs)

val climber =
    Climber(
        when (CURRENT_MODE) {
            Mode.REAL -> ClimberIOReal()
            Mode.SIM -> ClimberIOSim()
            Mode.REPLAY ->
                object : ClimberIO {
                    override var inputs = LoggedClimberInputs()
                }
        }
    )

val elevator =
    Elevator(
        when (CURRENT_MODE) {
            Mode.REAL -> ElevatorIOReal()
            Mode.SIM -> ElevatorIOSim()
            Mode.REPLAY ->
                object : ElevatorIO {
                    override var inputs = LoggedElevatorInputs()
                }
        }
    )

val gripper =
    Gripper(
        when (CURRENT_MODE) {
            Mode.REAL -> GripperIOReal()
            Mode.SIM -> GripperIOSim()
            Mode.REPLAY ->
                object : GripperIO {
                    override var inputs = LoggedGripperInputs()
                }
        }
    )

val extender =
    Extender(
        when (CURRENT_MODE) {
            Mode.REAL -> ExtenderIOReal()
            Mode.SIM -> ExtenderIOSim()
            Mode.REPLAY ->
                object : ExtenderIO {
                    override var inputs = LoggedExtenderInputs()
                }
        }
    )

val roller =
    Roller(
        when (CURRENT_MODE) {
            Mode.REAL -> RollerIOReal()
            Mode.SIM ->
                if (USE_MAPLE_SIM) RollerIOMapleSim(driveSimulation!!)
                else RollerIOSim()
            Mode.REPLAY ->
                object : RollerIO {
                    override var inputs = LoggedRollerInputs()
                }
        }
    )

val wrist =
    Wrist(
        when (CURRENT_MODE) {
            Mode.REAL -> WristIOReal()
            Mode.SIM -> WristIOSim()
            Mode.REPLAY ->
                object : WristIO {
                    override var inputs = LoggedWristInputs()
                }
        }
    )

val leds = LEDs()
