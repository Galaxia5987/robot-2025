package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.autonomous.A2R
import frc.robot.autonomous.A2R3RL
import frc.robot.autonomous.B1L
import frc.robot.autonomous.B1R
import frc.robot.autonomous.C6L
import frc.robot.autonomous.C6L5LR
import frc.robot.autonomous.S5L
import frc.robot.autonomous.S5R
import frc.robot.autonomous.alignScoreL1
import frc.robot.autonomous.alignScoreL2
import frc.robot.autonomous.alignScoreL3
import frc.robot.autonomous.alignScoreL4
import frc.robot.autonomous.alignToReefAlgae2
import frc.robot.autonomous.alignToReefAlgae3
import frc.robot.autonomous.pathFindC6L5LR
import frc.robot.autonomous.pathFindToSelectedFeeder
import frc.robot.autonomous.setFeederBasedOnAxis
import frc.robot.autonomous.setPoseBasedOnButton
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.subsystems.Visualizer
import frc.robot.subsystems.alignmentSetpointL4
import frc.robot.subsystems.blockedFeeder
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.elevator.MANUAL_CONTROL_VOLTAGE as ELEVATOR_MANUAL_CONTROL_VOLTAGE
import frc.robot.subsystems.feeder
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.intake.outtakeAlgae
import frc.robot.subsystems.intakeAlgaeToGripper
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l2algae
import frc.robot.subsystems.l2algaePickup
import frc.robot.subsystems.l3Manual
import frc.robot.subsystems.l3algae
import frc.robot.subsystems.l3algaePickup
import frc.robot.subsystems.l4
import frc.robot.subsystems.moveDefaultPosition
import frc.robot.subsystems.netAlgae
import frc.robot.subsystems.outtakeCoralAlignment
import frc.robot.subsystems.outtakeCoralManual
import frc.robot.subsystems.outtakeL1
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

    val driverController = CommandPS5Controller(0)
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
    val disableAlignment: Trigger? = heightController.button(12)
    val disablePathFinding = heightController.button(11)
    val disableFeederAlign = heightController.button(10)
    val shouldNet = heightController.button(8)
    val userButton = Trigger { RobotController.getUserButton() }

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
                { driverController.leftY * if (IS_RED) 1.0 else -1.0 },
                { driverController.leftX * if (IS_RED) 1.0 else -1.0 },
                { -driverController.rightX * 0.8 }
            )

        climber.defaultCommand =
            climber.powerControl {
                MathUtil.applyDeadband(operatorController.leftY, 0.15)
            }
    }

    private fun configureButtonBindings() {
        userButton
            .and(RobotModeTriggers.disabled())
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

        // reset swerve
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

        // align score
        driverController
            .cross()
            .whileTrue(alignScoreL1().onlyIf(disableAlignment!!.negate()))
        driverController
            .square()
            .whileTrue(alignScoreL2().onlyIf(disableAlignment.negate()))
            .onFalse(
                moveDefaultPosition(false, { false }).onlyIf {
                    moveDefaultPosition(false, { false }).isScheduled.not()
                }
            )
        driverController
            .circle()
            .whileTrue(alignScoreL3().onlyIf(disableAlignment.negate()))
        driverController
            .triangle()
            .whileTrue(alignScoreL4().onlyIf(disableAlignment.negate()))

        // manual score
        driverController.cross().and(disableAlignment).onTrue(l1()).onFalse(outtakeL1())
        driverController
            .square()
            .and(disableAlignment)
            .onTrue(l2())
            .onFalse(outtakeCoralAlignment(true))
        driverController
            .circle()
            .and(disableAlignment)
            .onTrue(l3Manual())
            .onFalse(outtakeCoralAlignment())
        driverController
            .triangle()
            .and(disableAlignment)
            .onTrue(l4())
            .onFalse(outtakeCoralManual())

        // intake buttons
        driverController.R1().and(shouldNet.negate()).whileTrue(intakeAlgae())
        driverController
            .R1()
            .and(shouldNet)
            .onTrue(intakeAlgaeToGripper(driverController.R1().negate()))
        driverController
            .L1()
            .onTrue(outtakeAlgae(driverController.L1().negate()))

        // gripper control
        driverController.R2().whileTrue(gripper.intake())
        driverController.L2().whileTrue(gripper.outtake(true))

        // remove algae
        operatorController
            .x()
            .and(shouldNet.negate())
            .onTrue(l2algae(operatorController.x().negate()))
        operatorController
            .b()
            .and(shouldNet.negate())
            .onTrue(l3algae(operatorController.b().negate()))
        heightController
            .button(2)
            .and(shouldNet.negate())
            .onTrue(l2algae(heightController.button(2).negate()))
        heightController
            .button(3)
            .and(shouldNet.negate())
            .onTrue(l3algae(heightController.button(3).negate()))

        // align pick algae from reef
        operatorController.x()
            .whileTrue(alignToReefAlgae2().onlyIf(disableAlignment.negate()))
        operatorController.b()
            .whileTrue(alignToReefAlgae3().onlyIf(disableAlignment.negate()))
        heightController
            .button(2)
            .whileTrue(alignToReefAlgae2().onlyIf(disableAlignment.negate()))
        heightController
            .button(3)
            .whileTrue(alignToReefAlgae3().onlyIf(disableAlignment.negate()))

        // manual pick algae from reef
        operatorController
            .x()
            .and(disableAlignment)
            .and(shouldNet)
            .onTrue(l2algaePickup())
            .onFalse(wrist.max())
        operatorController
            .b()
            .and(disableAlignment)
            .and(shouldNet)
            .onTrue(l3algaePickup())
            .onFalse(wrist.max())
        heightController
            .button(2)
            .and(disableAlignment)
            .and(shouldNet)
            .onTrue(l2algaePickup())
            .onFalse(wrist.max())
        heightController
            .button(3)
            .and(disableAlignment)
            .and(shouldNet)
            .onTrue(l3algaePickup())
            .onFalse(wrist.max())

        // net
        poseController
            .axisGreaterThan(0, 0.0)
            .onTrue(netAlgae(poseController.axisGreaterThan(0, 0.0).negate()))

        // feeder auto
        (operatorController.start().or(operatorController.back()))
            .and(disableFeederAlign.negate())
            .whileTrue(pathFindToSelectedFeeder())
        (heightController.button(1).or(heightController.button(4)))
            .and(disableFeederAlign.negate())
            .whileTrue(pathFindToSelectedFeeder())

        // feeder manual
        operatorController
            .start()
            .onTrue(feeder(Trigger { true }, disableAlignment))
        heightController
            .button(1)
            .onTrue(feeder(Trigger { true }, disableAlignment))
        operatorController
            .back()
            .onTrue(blockedFeeder(Trigger { true }, disableAlignment))
        heightController
            .button(4)
            .onTrue(blockedFeeder(Trigger { true }, disableAlignment))

        // auto lower climb
        Trigger { DriverStation.getMatchTime() <= 20 }
            .onTrue(climber.powerControl { 1.0 }.withTimeout(2.3))

        // overrides and resets
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

        disableAlignment.onTrue(wrist.retract())

        // test controller
        testController.a().whileTrue(runAllBits())

        // tuning buttons
        driverController.povDown().onTrue(elevator.tuningPosition())
        driverController.povRight().onTrue(wrist.tuningAngle())
        driverController.povLeft().onTrue(gripper.slowOuttake(true))

        // pose selector
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
        poseController.axisLessThan(0, -0.01).onTrue(setFeederBasedOnAxis(0))
        poseController.axisLessThan(1, -0.01).onTrue(setFeederBasedOnAxis(1))
    }

    fun getAutonomousCommand(): Command = autoChooser.selected

    private fun registerAutoCommands() {
        val namedCommands =
            mapOf(
                "MoveL4" to alignmentSetpointL4(),
                "MoveFeeder" to moveDefaultPosition(true, { false }),
                "ResetCoral" to
                    (wrist.feeder().alongWith(gripper.intake())).withTimeout(
                        0.25
                    ),
                "UseLocalEstimation" to
                    Commands.runOnce({ swerveDrive.useLocalInAuto = true }),
                "DisableLocalEstimation" to
                    Commands.runOnce({ swerveDrive.useLocalInAuto = false })
            )

        NamedCommands.registerCommands(namedCommands)

        autoChooser.setDefaultOption("B1L", B1L())
        autoChooser.addOption("None", Commands.none())
        autoChooser.addOption("B1L", B1L())
        autoChooser.addOption(
            "CalibrationPath",
            AutoBuilder.followPath(
                PathPlannerPath.fromPathFile("Calibration Path")
            )
        )
        autoChooser.addOption(
            "No spin calibration",
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("noSpin"))
        )
        autoChooser.addOption("B1R", B1R())
        autoChooser.addOption("C6L", C6L())
        autoChooser.addOption("S5L", S5L())
        autoChooser.addOption("S5R", S5R())
        autoChooser.addOption("C6L5LR", C6L5LR())
        autoChooser.addOption("A2R", A2R())
        autoChooser.addOption("A2R3RL", A2R3RL())
        autoChooser.addOption("Path find C6L5LR", pathFindC6L5LR())

        autoChooser.addOption("turnSysId", swerveDrive.runAllTurnSysID())
        autoChooser.addOption(
            "wheelRadiusCharacterization",
            DriveCommands.wheelRadiusCharacterization(swerveDrive)
        )
        autoChooser.addOption(
            "swerveFFCharacterization",
            DriveCommands.feedforwardCharacterization(swerveDrive)
        )
    }
}
