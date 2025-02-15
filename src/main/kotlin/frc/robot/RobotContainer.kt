package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.lib.startEnd
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

            cross().startEnd(::l1)
            square().startEnd(::l2)
            circle().startEnd(::l3)
            triangle().startEnd(::l4)

            L1().startEnd(::outtakeAlgae)
            R1().whileTrue(intakeAlgae())

            R2().whileTrue(gripper.intake())
            L2().whileTrue(gripper.outtake())
        }

        operatorController.apply {
            x().startEnd(::l2algae)
            b().startEnd(::l3algae)

            start().startEnd(::feeder)

            povDown().startEnd(elevator::reset)
            povUp().startEnd(extender::reset)
        }
    }

    fun getAutonomousCommand(): Command =
        DriveCommands.wheelRadiusCharacterization(swerveDrive)
}
