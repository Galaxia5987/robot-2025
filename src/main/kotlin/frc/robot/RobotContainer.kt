package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.subsystems.*
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.feeder
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.intake.outtakeAlgae
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l3
import frc.robot.subsystems.l4

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the [Robot] periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)
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
    val voltage = Units.Volts.of(1.0)

    init {

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
        visualizer = Visualizer()

        enableAutoLogOutputFor(this)
    }

    private fun getHeightCommand(): Command = Commands.none()

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
        fun createScoreCommandMap(): Map<Int, Command> {
            return mapOf(
                0 to Commands.none(),
                1 to
                    l1(
                        driverController.a().negate()
                    ), // TODO: Fill in correct trigger
                2 to
                    l2(
                        driverController.a().negate()
                    ), // TODO: Fill in correct trigger
                3 to
                    l3(
                        driverController.a().negate()
                    ), // TODO: Fill in correct trigger
                4 to
                    l4(
                        driverController.a().negate()
                    ), // TODO: Fill in correct trigger
                5 to feeder(Trigger { true }) // TODO: Fill in correct trigger
            )
        }

        driverController
            .back()
            .onTrue(
                Commands.runOnce(swerveDrive::resetGyro, swerveDrive)
                    .ignoringDisable(true)
            )

        driverController
            .a()
            .onTrue(
                Commands.defer(
                    {
                        createScoreCommandMap()[
                            networkTables.getTargetBranchPose()]
                    },
                    setOf(gripper, wrist, elevator)
                )
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

        driverController.a().onTrue(l1(driverController.a().negate()))
        driverController.x().onTrue(l2(driverController.x().negate()))
        driverController.b().onTrue(l3(driverController.b().negate()))
        driverController.y().onTrue(l4(driverController.y().negate()))
        driverController
            .start()
            .onTrue(feeder(driverController.start().negate()))
        driverController.rightTrigger().onTrue(gripper.intake())
        driverController.leftTrigger().onTrue(gripper.outtake())
        driverController
            .rightBumper()
            .whileTrue(intakeAlgae())
            .onFalse(outtakeAlgae(driverController.rightBumper().negate()))

        operatorController.a().onTrue(climber.fold())
        operatorController.b().onTrue(climber.unfold())
        operatorController
            .rightTrigger()
            .whileTrue(elevator.setVoltage(voltage))
    }

    fun getAutonomousCommand(): Command =
        DriveCommands.wheelRadiusCharacterization(swerveDrive)

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) =
            NamedCommands.registerCommand(name, command)
    }
}
