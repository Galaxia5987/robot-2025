package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorIOReal
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.gripper.GripperIOReal
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeIOReal

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

    val elevator: Elevator
    private val intake: Intake
    private val gripper: Gripper

    init {
        Elevator.initialize(ElevatorIOReal())
        Intake.initialize(IntakeIOReal())
        Gripper.initialize(GripperIOReal())
        elevator = Elevator.getInstance()
        intake = Intake.getInstance()
        gripper = Gripper.getInstance()
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
    }

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand =
            DriveCommands.joystickDriveAtAngle(
                swerveDrive,
                { driverController.leftY },
                { driverController.leftX },
                { swerveDrive.desiredHeading },
            )
                .alongWith(
                    swerveDrive.updateDesiredHeading {
                        -driverController.rightX
                    }
                )
        elevator.defaultCommand =
            elevator.setPower {
                driverController.rightTriggerAxis -
                        driverController.leftTriggerAxis
            }
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
        driverController
            .x()
            .onTrue(elevator.setPosition(Units.Centimeter.of(50.0)))
        driverController
            .a()
            .onTrue(elevator.setPosition(Units.Centimeter.of(0.0)))
        driverController
            .b()
            .whileTrue(elevator.setPosition(Units.Centimeter.of(100.0)))
        driverController
            .button(7)
            .onTrue(intake.setPosition(Units.Centimeter.of(0.0)))

        driverController
            .button(8)
            .onTrue(intake.setPosition(Units.Centimeter.of(-30.0)))
        driverController
            .leftBumper()
            .onTrue(gripper.setPosition(Units.Degrees.of(0.0)))
        driverController
            .rightBumper()
            .onTrue(gripper.setPosition(Units.Degrees.of(50.0)))
    }

    fun getAutonomousCommand(): Command =
        DriveCommands.wheelRadiusCharacterization(swerveDrive)

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) =
            NamedCommands.registerCommand(name, command)
    }
}
