package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.subsystems.*
import frc.robot.subsystems.drive.DriveCommands
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

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
    private val vision = frc.robot.vision
    private val climber = frc.robot.climber
    private val elevator = frc.robot.elevator
    private val gripper = frc.robot.gripper
    private val extender = frc.robot.extender
    private val roller = frc.robot.roller
    private val wrist = frc.robot.wrist
    val visualizer: Visualizer

    private val autoChooser = LoggedDashboardChooser<Command>("Auto Chooser")

    init {
        registerAutoRoutines()
        configureButtonBindings()
        configureDefaultCommands()

        SmartDashboard.putData(autoChooser.sendableChooser)

        visualizer =
            Visualizer(
                extender.position,
                { Units.Degrees.zero() },
                elevator.height,
                wrist.angle,
                { Units.Degrees.zero() },
                { Units.Degrees.zero() },
                { Units.Degrees.zero() }
            )

        enableAutoLogOutputFor(this)
    }

    private fun getDriveCommandReal(): Command =
        DriveCommands.joystickDriveAtAngle(
                swerveDrive,
                { driverController.leftY },
                { driverController.leftX },
                { swerveDrive.desiredHeading },
            )
            .alongWith(
                swerveDrive.updateDesiredHeading { -driverController.rightX }
            )

    private fun getDriveCommandSim(): Command =
        DriveCommands.joystickDrive(
            swerveDrive,
            { driverController.leftY },
            { driverController.leftX },
            { -driverController.rightX * 0.6 }
        )

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand =
            if (CURRENT_MODE == Mode.REAL) getDriveCommandReal()
            else getDriveCommandSim()
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

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun LoggedDashboardChooser<Command>.addAutoRoutine(routineName: String) {
        this.addOption(routineName, autoRoutines[routineName]?.cmd())
    }

    private fun registerAutoRoutines() {
        autoChooser.addDefaultOption("A leave", autoRoutines["A leave"]!!.cmd())
        autoChooser.addAutoRoutine("C6L5RL")
        autoChooser.addAutoRoutine("C5RL4R")
        autoChooser.addAutoRoutine("B1R2LR")
        autoChooser.addAutoRoutine("B1L6RL")
        autoChooser.addAutoRoutine("B1R")
        autoChooser.addAutoRoutine("B1L")
        autoChooser.addAutoRoutine("A3LR4L")
        autoChooser.addAutoRoutine("A2R3LR")
        autoChooser.addAutoRoutine("C leave")
        autoChooser.addAutoRoutine("B leave")
    }
}
