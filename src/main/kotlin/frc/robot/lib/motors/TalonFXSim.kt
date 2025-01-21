package frc.robot.lib.motors

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.math.differential.Derivative
import frc.robot.lib.motors.SimMotor.MotorSetpoint

class TalonFXSim : SimMotor {
    private val acceleration = Derivative()

    constructor(
        model: LinearSystem<N2?, N1?, N2?>,
        numMotors: Int,
        gearing: Double,
        conversionFactor: Double,
        motorType: TalonType
    ) : super(model, TalonType.getDCMotor(motorType, numMotors), gearing, conversionFactor)

    constructor(motor: DCMotor, gearing: Double, jKgMetersSquared: MomentOfInertia, conversionFactor: Double) : super(
        motor,
        jKgMetersSquared.`in`(Units.KilogramSquareMeters),
        gearing,
        conversionFactor
    )

    constructor(
        numMotors: Int,
        gearing: Double,
        jKgMetersSquared: MomentOfInertia,
        conversionFactor: Double,
        talonType: TalonType
    ) : super(
        TalonType.getDCMotor(talonType, numMotors),
        jKgMetersSquared.`in`(Units.KilogramSquareMeters),
        gearing,
        conversionFactor
    )

    override fun update(timestampSeconds: Double) {
        super.update(timestampSeconds)

        acceleration.update(velocity.`in`(Units.RotationsPerSecond), timestampSeconds)
    }

    fun setControl(request: DutyCycleOut) {
        setControl(VoltageOut(request.Output * 12))
    }

    fun setControl(request: VoltageOut) {
        voltageRequest = MotorSetpoint.simpleVoltage(request.Output)
    }

    fun setControl(request: PositionDutyCycle) {
        setControl(PositionVoltage(request.Position).withFeedForward(request.FeedForward * 12))
    }

    fun setControl(request: PositionVoltage) {
        voltageRequest =
            MotorSetpoint { controller.calculate(position, request.Position) + request.FeedForward }
    }

    fun setControl(request: VelocityDutyCycle) {
        setControl(VelocityVoltage(request.Velocity).withFeedForward(request.FeedForward * 12))
    }

    fun setControl(request: VelocityVoltage) {
        voltageRequest =
            MotorSetpoint {
                (controller.calculate(
                    velocity.`in`(Units.RotationsPerSecond),
                    request.Velocity
                )
                        + request.FeedForward)
            }
    }

    fun setControl(request: MotionMagicDutyCycle) {
        setControl(
            MotionMagicVoltage(request.Position).withFeedForward(request.FeedForward * 12)
        )
    }

    fun setControl(request: MotionMagicVoltage) {
        voltageRequest =
            MotorSetpoint {
                (profiledController.calculate(position, request.Position)
                        + request.FeedForward)
            }
    }

    val velocity: AngularVelocity
        get() = Units.Rotation.per(Units.Minutes)
            .of(motorSim.angularVelocityRPM) * conversionFactor

    val position: Double
        get() = motorSim.angularPositionRotations * conversionFactor

    val appliedCurrent: Current
        get() = Units.Amps.of(motorSim.currentDrawAmps)

    val appliedVoltage: Voltage
        get() = Units.Volts.of(motorSim.inputVoltage)

    fun getAcceleration(): Double {
        return acceleration.get()
    }
}
