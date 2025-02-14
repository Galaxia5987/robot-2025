package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.lib.withRotation
import frc.robot.subsystems.*
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.feeder
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.intake.outtakeAlgae
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l3
import frc.robot.subsystems.l4
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

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

    private val autoChooser = LoggedDashboardChooser<Command>("Auto Chooser")

    init {
        registerAutoRoutines()
        configureButtonBindings()
        configureDefaultCommands()
        SmartDashboard.putData(autoChooser.sendableChooser)
        visualizer = Visualizer()

        if (CURRENT_MODE == Mode.SIM)
            SimulatedArena.getInstance().resetFieldForAuto()

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
        driverController
            .create()
            .onTrue(
                Commands.runOnce(
                        {
                            swerveDrive::resetGyro
                            swerveDrive.resetOdometry(
                                swerveDrive.pose.withRotation(Rotation2d())
                            )
                        },
                        swerveDrive
                    )
                    .ignoringDisable(true)
            )

        driverController.cross().onTrue(l1(driverController.cross().negate()))
        driverController.square().onTrue(l2(driverController.square().negate()))
        driverController.circle().onTrue(l3(driverController.circle().negate()))
        driverController
            .triangle()
            .onTrue(l4(driverController.triangle().negate()))
        driverController.R1().whileTrue(intakeAlgae())
        driverController
            .L1()
            .onTrue(outtakeAlgae(driverController.L1().negate()))
        driverController.R2().whileTrue(gripper.intake())
        driverController.L2().whileTrue(gripper.outtake())

        operatorController.x().onTrue(l2algae(operatorController.x().negate()))
        operatorController.b().onTrue(l3algae(operatorController.b().negate()))
        operatorController
            .start()
            .onTrue(feeder(operatorController.start().negate()))
        operatorController
            .back()
            .onTrue(blockedFeeder(operatorController.back().negate()))
        operatorController
            .povDown()
            .onTrue(elevator.reset(operatorController.povDown().negate()))
        operatorController
            .povUp()
            .onTrue(extender.reset(operatorController.povUp().negate()))
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun LoggedDashboardChooser<Command>.addAutoRoutine(
        routineName: String
    ) {
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
