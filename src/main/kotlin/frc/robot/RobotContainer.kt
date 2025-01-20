package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.Visualizer
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorIOSim
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.gripper.GripperIOSim
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderIOSim
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.intake.roller.RollerIOSim
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristIOSim

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the [Robot] periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandPS5Controller(1)
    private val testController = CommandXboxController(2)

    private val swerveDrive = frc.robot.swerveDrive
    private val elevator = Elevator(ElevatorIOSim())
    private val wrist = Wrist(WristIOSim())
    private val gripper = Gripper(GripperIOSim())
    private val intakeExtender = Extender(ExtenderIOSim())
    private val intakeRoller = Roller(RollerIOSim())

    private val visualizer: Visualizer

    init {
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
        visualizer =
            Visualizer(
                intakeExtender.position,
                { Units.Degrees.zero() },
                elevator.position,
                wrist.angle,
                { Units.Degrees.zero() },
                { Units.Degrees.zero() },
                { Units.Degrees.zero() }
            )
    }

    fun getVisualizerPoses() = visualizer.visualizeSubsystems()

    private fun getDriveCommandReal(): Command = DriveCommands.joystickDriveAtAngle(
        swerveDrive,
        { driverController.leftY },
        { driverController.leftX },
        { swerveDrive.desiredHeading },
    )
        .alongWith(
            swerveDrive.updateDesiredHeading {
                -driverController.rightX
            })

    private fun getDriveCommandSim(): Command = DriveCommands.joystickDrive(
        swerveDrive,
        { driverController.leftY },
        { driverController.leftX },
        { -driverController.rightX * 0.6 })

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand = if (CURRENT_MODE == Mode.REAL) getDriveCommandReal() else getDriveCommandSim()

    }

    private fun configureButtonBindings() {
        driverController
            .y()
            .onTrue(
                Commands.runOnce(swerveDrive::resetGyro, swerveDrive)
                    .ignoringDisable(true)
            )

        driverController
            .povUp()
            .whileTrue(swerveDrive.setDesiredHeading(Rotation2d.kZero))
        driverController
            .povRight()
            .whileTrue(swerveDrive.setDesiredHeading(Rotation2d.kCW_90deg))
        driverController
            .povDown()
            .whileTrue(swerveDrive.setDesiredHeading(Rotation2d.k180deg))
        driverController
            .povLeft()
            .whileTrue(swerveDrive.setDesiredHeading(Rotation2d.kCCW_90deg))
    }

    fun getAutonomousCommand(): Command =
        DriveCommands.wheelRadiusCharacterization(swerveDrive)

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) =
            NamedCommands.registerCommand(name, command)
    }
}
