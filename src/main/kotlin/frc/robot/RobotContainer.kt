package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.autonomous.*
import frc.robot.compositions.autonomous.selectedScorePose
import frc.robot.compositions.autonomous.setPoseBasedOnButton
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.then
import frc.robot.subsystems.*
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.intake.outtakeAlgae
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

    val visualizer: Visualizer
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

    private fun configureButtonBox() {
        val buttonBox = CommandGenericHID(4)
        (1..12).zip('A'..'L').forEach { (buttonNumber, location) ->
            buttonBox.button(buttonNumber).onTrue( { selectedScorePose = location } )
        }
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
                MathUtil.applyDeadband(
                    operatorController.leftY + poseController.getRawAxis(0),
                    0.15
                )
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
            .onFalse(
                moveDefaultPosition(true, { false }).onlyIf {
                    moveDefaultPosition(true, { false }).isScheduled.not()
                }
            )
        driverController
            .triangle()
            .whileTrue(alignScoreL4().onlyIf(disableAlignment.negate()))
            .onFalse(
                moveDefaultPosition(true, { false }).onlyIf {
                    moveDefaultPosition(true, { false }).isScheduled.not()
                }
            )

        driverController
            .cross()
            .onTrue(l1())
            .onFalse(outtakeL1())
        driverController
            .square()
            .and(disableAlignment)
            .onTrue(l2())
            .onFalse(outtakeCoralAndDriveBack(false, isReverse = true))
        driverController
            .circle()
            .and(disableAlignment)
            .onTrue(l3Manual())
            .onFalse(outtakeCoralAndDriveBack(true))
        driverController
            .triangle()
            .and(disableAlignment)
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
                feeder(operatorController.start().negate(), disableAlignment)
            )
        heightController
            .button(1)
            .onTrue(
                feeder(operatorController.start().negate(), disableAlignment)
            )
        operatorController
            .back()
            .onTrue(
                blockedFeeder(
                    operatorController.back().negate(),
                    disableAlignment
                )
            )
        heightController
            .button(4)
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
                "MoveL4" to alignmentSetpointL4(),
                "MoveFeeder" to moveDefaultPosition(true, { false }),
                "ResetCoral" to
                        (feeder(Trigger { true }, { false })
                                then WaitCommand(0.4)
                                then gripper.intake().withTimeout(0.25)),
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
        autoChooser.addOption("B1R", B1R())
        autoChooser.addOption("C6L", C6L())
        autoChooser.addOption("C6L5LR", C6L5LR())
        autoChooser.addOption("A2R", A2R())
        autoChooser.addOption("A2R3RL", A2R3RL())
    }
}

private fun Trigger.onTrue(function: () -> Unit) = onTrue(Commands.runOnce(function))
