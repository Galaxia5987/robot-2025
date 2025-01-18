package frc.robot.subsystems.climber

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.SolenoidSim
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class ClimberIOSim : ClimberIO {
    override var inputs: LoggedInputClimber = LoggedInputClimber()
    var motor =
        TalonFXSim(2, GEAR_RATIO, MOMENT_OF_INERTIA_MAIN.`in`(Units.KilogramSquareMeters), 1.0, TalonType.KRAKEN)
    var servo1 = SolenoidSim(PneumaticsModuleType.REVPH, 0)
    var servo2 = SolenoidSim(PneumaticsModuleType.REVPH, 0)
    val lockMotor = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getBag(1),
            MOMENT_OF_INERTIA_LOCK.`in`(Units.KilogramSquareMeters),
            1.0
        ),
        DCMotor.getBag(1)
    )
    var dutyCycle = DutyCycleOut(0.0)
    var positionControler = PositionVoltage(0.0)


    override fun setLatchPosition(position: Double) {
        if (Double.equals(OPEN_LATCH_POSITION)) {
            listOf(servo2, servo1).forEach { it.output = true }
        } else {
            listOf(servo2, servo1).forEach { it.output = false }
        }
    }

    override fun lock() {
        lockMotor.setAngle(LOCK_ANGLE.`in`(Units.Radians))
    }

    override fun unlock() {
        lockMotor.setAngle(UNLOCK_ANGLE.`in`(Units.Radians))
    }

    override fun setPower(power: Double) {
        motor.setControl(dutyCycle.withOutput(power))
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(positionControler.withPosition(angle))
    }

    override fun updateInput() {
        motor.update(Timer.getFPGATimestamp())
        lockMotor.update(Timer.getFPGATimestamp())
        inputs.angle = Units.Rotations.of(motor.position)
        inputs.appliedVoltage = Units.Volt.of(motor.appliedVoltage)
        if (servo1.output) {
            inputs.latchPosition = OPEN_LATCH_POSITION
        } else {
            inputs.latchPosition = CLOSE_LATCH_POSITION
        }
    }
}