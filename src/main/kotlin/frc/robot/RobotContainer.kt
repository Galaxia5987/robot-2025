package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.autonomous.*
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.subsystems.*
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.elevator.MANUAL_CONTROL_VOLTAGE as ELEVATOR_MANUAL_CONTROL_VOLTAGE
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.intake.outtakeAlgae
import frc.robot.subsystems.wrist.MANUAL_CONTROL_VOLTAGE as WRIST_MANUAL_CONTROL_VOLTAGE
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
    private val heightController = CommandGenericHID(3)
    private val poseController = CommandGenericHID(4)

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


        if (CURRENT_MODE == Mode.SIM && USE_MAPLE_SIM)
            SimulatedArena.getInstance().resetFieldForAuto()

        enableAutoLogOutputFor(this)
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand =
            DriveCommands.joystickDrive(
                swerveDrive,
                { driverController.leftY },
                { driverController.leftX },
                { -driverController.rightX * 0.6 }
            )

        climber.defaultCommand =
            climber.powerControl {
                MathUtil.applyDeadband(operatorController.leftY, 0.15)
            }
    }

    private fun configureButtonBindings() {
        driverController
            .create()
            .onTrue(
                Commands.runOnce(
                        {
                            if (IS_RED) swerveDrive.resetGyro(Rotation2d.kZero)
                            else swerveDrive.resetGyro(Rotation2d.k180deg)
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
        driverController.L2().whileTrue(gripper.outtake(true))

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
        operatorController
            .povLeft()
            .onTrue(Commands.runOnce({ isLeft = { true } }))
        operatorController
            .povRight()
            .onTrue(Commands.runOnce({ isLeft = { false } }))
        operatorController
            .rightTrigger()
            .whileTrue(elevator.setVoltage(ELEVATOR_MANUAL_CONTROL_VOLTAGE))
        operatorController
            .leftTrigger()
            .whileTrue(elevator.setVoltage(-ELEVATOR_MANUAL_CONTROL_VOLTAGE))
        operatorController
            .rightBumper()
            .whileTrue(wrist.setVoltage(WRIST_MANUAL_CONTROL_VOLTAGE))
        operatorController
            .leftBumper()
            .whileTrue(wrist.setVoltage(WRIST_MANUAL_CONTROL_VOLTAGE))

        testController.a().onTrue(intakeBit(testController.a().negate()))
        testController.y().onTrue(feederL4Bit(testController.y().negate()))

        val buttonMappings =
            listOf(
                9, // L1
                10, // R1
                11, // R2
                8, // L2
                4, // L3
                12, // R3
                7, // L4
                1, // R4
                6, // L5
                2, // R5
                5, // L6
                3 // R6
            )

        buttonMappings.forEach { buttonId ->
            poseController
                .button(buttonId)
                .onTrue(setPoseBasedOnButton(buttonId))
        }
    }

    fun getAutonomousCommand(): Command = dumbAuto()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) =
            NamedCommands.registerCommand(name, command)
    }
}
