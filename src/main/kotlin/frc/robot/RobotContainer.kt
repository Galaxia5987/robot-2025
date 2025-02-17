package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.lib.withRotation
import frc.robot.subsystems.drive.DriveCommands
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the [Robot] periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and trigger mappings) should be declared here.
 */
object RobotContainer {

    private val driverController = CommandPS5Controller(0)
    private val operatorController = CommandXboxController(1)
    private val testController = CommandXboxController(2)

    init {

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        enableAutoLogOutputFor(this)
    }

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand =
        DriveCommands.joystickDrive(
            swerveDrive,
            { driverController.leftY },
            { driverController.leftX },
            { -driverController.rightX * 0.6 }
        )
    }

    private fun configureButtonBindings() {
        driverController
            .create()
            .onTrue(
                Commands.runOnce(swerveDrive::resetGyro)
                    .ignoringDisable(true)
            )
    }

    fun getAutonomousCommand(): Command =
        DriveCommands.wheelRadiusCharacterization(swerveDrive)

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) =
            NamedCommands.registerCommand(name, command)
    }
}
