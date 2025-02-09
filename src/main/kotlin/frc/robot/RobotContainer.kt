package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.subsystems.*
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.intake.outtakeAlgae
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

    val visualizer = Visualizer()

    init {
        configureButtonBindings()
        configureDefaultCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

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
        driverController.apply {
            create()
                .onTrue(
                    Commands.runOnce(swerveDrive::resetGyro)
                        .ignoringDisable(true)
                )
            (listOf(
                    cross() to ::l1,
                    square() to ::l2,
                    circle() to ::l3,
                    triangle() to ::l4,
                    L1() to ::outtakeAlgae
                ))
                .forEach { (button, command) ->
                    button.onTrue(command(button.negate()))
                }
            R1().whileTrue(intakeAlgae())

            gripper.apply {
                R2().whileTrue(intake())
                L2().whileTrue(outtake())
            }
        }

        operatorController.apply {
            (listOf(
                    x() to ::l2algae,
                    b() to ::l3algae,
                    start() to ::feeder,
                ))
                .forEach { (button, command) ->
                    button.onTrue(command(button.negate()))
                }
            povDown().apply { onTrue(elevator.reset(negate())) }
            povUp().apply { onTrue(extender.reset(negate())) }
        }
    }

    fun getAutonomousCommand(): Command =
        DriveCommands.wheelRadiusCharacterization(swerveDrive)
}
