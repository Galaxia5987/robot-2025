package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Timer
import frc.robot.driveSimulation
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly

class RollerIOSim(driveTrainSimulation: AbstractDriveTrainSimulation) :
    RollerIO {
    override val inputs = LoggedRollerInputs()
    private val intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Algae",
            driveTrainSimulation,
            Units.Meters.of(0.64),
            Units.Meters.of(0.35),
            IntakeSimulation.IntakeSide.FRONT,
            1
        )
    private val motor =
        TalonFXSim(
            1,
            GEAR_RATIO,
            MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
            1.0,
            TalonType.FALCON
        )
    private val controlRequest = VoltageOut(0.0)

    private fun outtakeGamePieceIfNeed() {
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    ReefscapeAlgaeOnFly(
                        driveSimulation!!.simulatedDriveTrainPose.translation,
                        Translation2d(
                            Units.Meters.of(0.3),
                            Units.Meters.zero()
                        ),
                        driveSimulation
                            .driveTrainSimulatedChassisSpeedsFieldRelative,
                        driveSimulation.simulatedDriveTrainPose.rotation,
                        Units.Meter.of(0.4),
                        Units.MetersPerSecond.of(3.0),
                        Units.Degrees.of(0.0)
                    )
                )
        }
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(controlRequest.withOutput(voltage))
        if (voltage != Units.Volts.zero()) {
            intakeSimulation.startIntake()
        } else {
            intakeSimulation.stopIntake()
            outtakeGamePieceIfNeed()
        }
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.appliedVoltage = motor.appliedVoltage
    }
}
