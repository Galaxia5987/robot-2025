package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.drive.DriveCommands
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

    init {
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
