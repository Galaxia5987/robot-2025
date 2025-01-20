package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
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
    private var servo1 = SolenoidSim(PneumaticsModuleType.REVPH, LATCH_SERVO_ID)
    private var servo2 =
        SolenoidSim(PneumaticsModuleType.REVPH, FOLLOW_LATCH_SERVO_ID)
    private val stopperMotor =
        TalonFXSim(
            DCMotor.getBag(1),
            STOPPER_GEAR_RATIO,
            MOMENT_OF_INERTIA_LOCK.`in`(Units.KilogramSquareMeters),
            1.0
        )
    private var voltageControl = VoltageOut(0.0)
    private var positionController = PositionVoltage(0.0)

    override fun setLatchPosition(position: Distance) {
        if (Double.equals(OPEN_LATCH_POSITION)) {
            listOf(servo2, servo1).forEach { it.output = true }
        } else {
            listOf(servo2, servo1).forEach { it.output = false }
        }
    }

    override fun closeStopper() {
        stopperMotor.setControl(voltageControl.withOutput(LOCK_POWER * 12))
    }

    override fun openStopper() {
        stopperMotor.setControl(voltageControl.withOutput(UNLOCK_POWER * 12))
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
        if (servo1.output) {
            inputs.latchPosition = OPEN_LATCH_POSITION
        } else {
            inputs.latchPosition = CLOSE_LATCH_POSITION
        }
    }
}
