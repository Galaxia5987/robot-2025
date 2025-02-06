package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.SolenoidSim
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class ClimberIOSim : ClimberIO {
    override var inputs: LoggedClimberInputs = LoggedClimberInputs()

    private var motor =
        TalonFXSim(
            2,
            GEAR_RATIO,
            MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
            1.0,
            TalonType.KRAKEN_FOC
        )

    private var latchServo =
        SolenoidSim(PneumaticsModuleType.REVPH, LATCH_SERVO_ID)

    private val stopperMotor =
        TalonFXSim(
            DCMotor.getBag(1),
            STOPPER_GEAR_RATIO,
            MOMENT_OF_INERTIA_LOCK.`in`(Units.KilogramSquareMeters),
            1.0
        )

    private var voltageControl = VoltageOut(0.0)
    private var powerControl = DutyCycleOut(0.0)
    private var positionController = PositionVoltage(0.0)

    init {
        motor.setController(PIDController(GAINS.kP, GAINS.kI, GAINS.kD))
    }

    override fun setLatchPosition(position: Distance) {
        latchServo.output = position == OPEN_LATCH_POSITION
    }

    override fun setStopperPower(power: Double) {
        stopperMotor.setControl(powerControl.withOutput(power))
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageControl.withOutput(voltage))
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(positionController.withPosition(angle))
    }

    override fun updateInput() {
        motor.update(Timer.getFPGATimestamp())
        stopperMotor.update(Timer.getFPGATimestamp())
        inputs.angle = Units.Rotations.of(motor.position)
        inputs.appliedVoltage = motor.appliedVoltage
        inputs.latchPosition =
            if (latchServo.output) OPEN_LATCH_POSITION else CLOSE_LATCH_POSITION
        inputs.angularVelocity = motor.velocity
    }
}
