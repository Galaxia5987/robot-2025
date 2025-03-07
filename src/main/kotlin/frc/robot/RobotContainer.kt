package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.autonomous.A2R
import frc.robot.autonomous.A2R3RL
import frc.robot.autonomous.B1L
import frc.robot.autonomous.B1R
import frc.robot.autonomous.C6L
import frc.robot.autonomous.C6L5LR
import frc.robot.autonomous.alignScoreL1
import frc.robot.autonomous.alignScoreL2
import frc.robot.autonomous.alignScoreL3
import frc.robot.autonomous.alignScoreL4
import frc.robot.autonomous.setPoseBasedOnButton
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.subsystems.Visualizer
import frc.robot.subsystems.blockedFeeder
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.feeder
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.intake.outtakeAlgae
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l2algae
import frc.robot.subsystems.l3
import frc.robot.subsystems.l3algae
import frc.robot.subsystems.l4
import frc.robot.subsystems.moveDefaultPosition
import frc.robot.subsystems.outtakeCoralAndDriveBack
import frc.robot.subsystems.outtakeL1
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import frc.robot.subsystems.elevator.MANUAL_CONTROL_VOLTAGE as ELEVATOR_MANUAL_CONTROL_VOLTAGE
import frc.robot.subsystems.wrist.MANUAL_CONTROL_VOLTAGE as WRIST_MANUAL_CONTROL_VOLTAGE

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
    val disableAlignment = heightController.button(12)

    val autoChooser = AutoBuilder.buildAutoChooser()

    init {

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
        visualizer = Visualizer()

        SmartDashboard.putData(autoChooser)

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
                        swerveDrive.resetGyroBasedOnAlliance(
                            Rotation2d.kZero
                        )
                    },
                    swerveDrive
                )
                    .ignoringDisable(true)
            )

        driverController
            .cross()
            .whileTrue(alignScoreL1().onlyIf(disableAlignment.negate()))
        driverController
            .square()
            .whileTrue(alignScoreL2().onlyIf(disableAlignment.negate()))
        driverController
            .circle()
            .whileTrue(alignScoreL3().onlyIf(disableAlignment.negate()))
        driverController
            .triangle()
            .whileTrue(alignScoreL4().onlyIf(disableAlignment.negate()))

        driverController
            .cross()
            .and(disableAlignment)
            .onTrue(l1())
            .onFalse(outtakeL1())
        driverController
            .square().and(disableAlignment)
            .onTrue(l2())
            .onFalse(outtakeCoralAndDriveBack(false, isReverse = true))
        driverController
            .circle().and(disableAlignment)
            .onTrue(l3())
            .onFalse(outtakeCoralAndDriveBack(true))
        driverController
            .triangle().and(disableAlignment)
            .onTrue(l4())
            .onFalse(outtakeCoralAndDriveBack(true))

        driverController.R1().whileTrue(intakeAlgae())
        driverController
            .L1()
            .onTrue(outtakeAlgae(driverController.L1().negate()))
        driverController.R2().whileTrue(gripper.intake())
        driverController.L2().whileTrue(gripper.outtake(true))

        operatorController.x().onTrue(l2algae(operatorController.x().negate()))
        operatorController.b().onTrue(l3algae(operatorController.b().negate()))
        heightController
            .button(2)
            .onTrue(l2algae(heightController.button(2).negate()))
        heightController
            .button(3)
            .onTrue(l3algae(heightController.button(3).negate()))
        operatorController
            .start()
            .onTrue(
                feeder(
                    operatorController.start().negate(),
                    disableAlignment
                )
            )
        heightController
            .button(1)
            .onTrue(
                feeder(
                    operatorController.start().negate(),
                    disableAlignment
                )
            )
        operatorController
            .back()
            .onTrue(
                blockedFeeder(
                    operatorController.back().negate(),
                    disableAlignment
                )
            )
        operatorController
            .povDown()
            .onTrue(elevator.reset(operatorController.povDown().negate()))
        operatorController
            .povUp()
            .onTrue(extender.reset(operatorController.povUp().negate()))
        operatorController
            .povRight()
            .onTrue(wrist.reset(operatorController.povRight().negate()))
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
            .whileTrue(wrist.setVoltage(-WRIST_MANUAL_CONTROL_VOLTAGE))

        disableAlignment.onTrue(wrist.l1())

        testController.a().whileTrue(runAllBits())

        driverController.povDown().onTrue(elevator.tuningPosition())
        driverController.povRight().onTrue(wrist.tuningAngle())
        driverController.povLeft().onTrue(gripper.slowOuttake(true))

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

    fun getAutonomousCommand(): Command = autoChooser.selected

    private fun registerAutoCommands() {
        val namedCommands =
            mapOf(
                "MoveL4" to l4(),
                "MoveFeeder" to moveDefaultPosition(true),
            )

        NamedCommands.registerCommands(namedCommands)

        autoChooser.setDefaultOption("B1L", B1L())
        autoChooser.addOption("None", Commands.none())
        autoChooser.addOption("B1L", B1L())
        autoChooser.addOption("B1R", B1R())
        autoChooser.addOption("C6L", C6L())
        autoChooser.addOption("C6L5LR", C6L5LR())
        autoChooser.addOption("A2R", A2R())
        autoChooser.addOption("A2R3RL", A2R3RL())
    }
}
