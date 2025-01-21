package frc.robot

import frc.robot.subsystems.climber.Climber
import frc.robot.subsystems.climber.ClimberIO
import frc.robot.subsystems.climber.ClimberIOReal
import frc.robot.subsystems.climber.ClimberIOSim
import frc.robot.subsystems.climber.LoggedClimberInputs
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.GyroIO
import frc.robot.subsystems.drive.GyroIONavX
import frc.robot.subsystems.drive.ModuleIO
import frc.robot.subsystems.drive.ModuleIOSim
import frc.robot.subsystems.drive.ModuleIOTalonFX
import frc.robot.subsystems.drive.TunerConstants
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
import frc.robot.subsystems.intake.roller.LoggedRollerInputs
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.intake.roller.RollerIO
import frc.robot.subsystems.intake.roller.RollerIOReal
import frc.robot.subsystems.intake.roller.RollerIOSim
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim
import frc.robot.subsystems.wrist.LoggedWristInputs
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristIO
import frc.robot.subsystems.wrist.WristIOReal
import frc.robot.subsystems.wrist.WristIOSim

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

val subsystems =
    when (CURRENT_MODE) {
        Mode.REAL -> {
            mapOf(
                "Climber" to Climber(ClimberIOReal()),
                "Elevator" to Elevator(ElevatorIOReal()),
                "Gripper" to Gripper(GripperIOReal()),
                "Extender" to Extender(ExtenderIOReal()),
                "Roller" to Roller(RollerIOReal()),
                "Wrist" to Wrist(WristIOReal())
            )
        }

        Mode.SIM -> {
            mapOf(
                "Climber" to Climber(ClimberIOSim()),
                "Elevator" to Elevator(ElevatorIOSim()),
                "Gripper" to Gripper(GripperIOSim()),
                "Extender" to Extender(ExtenderIOSim()),
                "Roller" to Roller(RollerIOSim()),
                "Wrist" to Wrist(WristIOSim())
            )
        }

        Mode.REPLAY -> {
            mapOf(
                "Climber" to
                        Climber(
                            object : ClimberIO {
                                override var inputs = LoggedClimberInputs()
                            }
                        ),
                "Elevator" to
                        Elevator(
                            object : ElevatorIO {
                                override var inputs = LoggedElevatorInputs()
                            }
                        ),
                "Gripper" to
                        Gripper(
                            object : GripperIO {
                                override var inputs = LoggedGripperInputs()
                            }
                        ),
                "Extender" to
                        Extender(
                            object : ExtenderIO {
                                override var inputs = LoggedExtenderInputs()
                            }
                        ),
                "Roller" to
                        Roller(
                            object : RollerIO {
                                override var inputs = LoggedRollerInputs()
                            }
                        ),
                "Wrist" to
                        Wrist(
                            object : WristIO {
                                override var inputs = LoggedWristInputs()
                            }
                        )
            )
        }
    }
