package frc.robot

import frc.robot.subsystems.climber.*
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.elevator.*
import frc.robot.subsystems.gripper.*
import frc.robot.subsystems.intake.extender.*
import frc.robot.subsystems.intake.roller.*
import frc.robot.subsystems.leds.LEDs
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim
import frc.robot.subsystems.wrist.*

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
            Mode.SIM -> RollerIOSim()
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
