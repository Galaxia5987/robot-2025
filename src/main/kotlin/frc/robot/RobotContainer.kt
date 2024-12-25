package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.ControllerInputs.driverController
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.getGyroIO
import frc.robot.subsystems.drive.getSwerveModuleIOs

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val swerveDrive: Drive
    private val testController = CommandXboxController(2)

    init {
        swerveDrive = Drive(getGyroIO(), getSwerveModuleIOs())

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
    }

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand = DriveCommands.joystickDriveAtAngle(
            swerveDrive,
            { driverController().leftY },
            { driverController().leftX },
            { swerveDrive.desiredHeading }
        ).alongWith(swerveDrive.updateDesiredHeading { -driverController().rightX })
    }

    private fun configureButtonBindings() {
        driverController().y().onTrue(
            Commands.runOnce({
                swerveDrive.pose = Pose2d(swerveDrive.pose.translation, Rotation2d())
            }, swerveDrive)
                .ignoringDisable(true)
        )

        driverController().povUp().whileTrue(swerveDrive.setDesiredHeading(Rotation2d.fromDegrees(0.0)))
        driverController().povRight().whileTrue(swerveDrive.setDesiredHeading(Rotation2d.fromDegrees(90.0)))
        driverController().povDown().whileTrue(swerveDrive.setDesiredHeading(Rotation2d.fromDegrees(180.0)))
        driverController().povLeft().whileTrue(swerveDrive.setDesiredHeading(Rotation2d.fromDegrees(-90.0)))
    }

    fun getAutonomousCommand(): Command = DriveCommands.wheelRadiusCharacterization(swerveDrive)

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
