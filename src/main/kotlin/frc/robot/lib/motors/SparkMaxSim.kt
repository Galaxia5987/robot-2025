package frc.robot.lib.motors

import com.revrobotics.spark.SparkBase.ControlType
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.MomentOfInertia

class SparkMaxSim : SimMotor {
    constructor(
        numMotors: Int,
        gearing: Double,
        momentOfInertia: MomentOfInertia,
        conversionFactor: Double
    ) : super(
        DCMotor.getNEO(numMotors),
        momentOfInertia.`in`(Units.KilogramSquareMeters),
        gearing,
        conversionFactor
    )

    constructor(
        motor: DCMotor,
        gearing: Double,
        momentOfInertia: MomentOfInertia,
        conversionFactor: Double
    ) : super(
        motor,
        momentOfInertia.`in`(Units.KilogramSquareMeters),
        gearing,
        conversionFactor
    )

    constructor(
        model: LinearSystem<N2, N1, N2>,
        numMotors: Int,
        gearing: Double,
        conversionFactor: Double
    ) : super(model, DCMotor.getNEO(numMotors), gearing, conversionFactor)

    fun set(speed: Double) {
        setInputVoltage(speed * 12.0)
    }

    fun setInputVoltage(voltage: Double) {
        voltageRequest = MotorSetpoint.simpleVoltage(voltage)
    }

    fun setReference(value: Double, ctrl: ControlType) {
        setReference(value, ctrl, 0.0)
    }

    private fun setInputVoltage(voltage: MotorSetpoint) {
        voltageRequest = voltage
    }

    fun setReference(value: Double, ctrl: ControlType, arbFeedforward: Double) {
        when (ctrl) {
            ControlType.kDutyCycle -> set(value)
            ControlType.kPosition ->
                setInputVoltage {
                    controller.calculate(position, value) + arbFeedforward
                }
            ControlType.kMAXMotionPositionControl ->
                setInputVoltage {
                    profiledController.calculate(position, value) +
                        arbFeedforward
                }
            ControlType.kVelocity ->
                setInputVoltage {
                    controller.calculate(velocity, value) + arbFeedforward
                }
            ControlType.kMAXMotionVelocityControl ->
                setInputVoltage {
                    profiledController.calculate(velocity, value) +
                        arbFeedforward
                }
            ControlType.kVoltage -> setInputVoltage(value)
            ControlType.kCurrent ->
                println("Can't use current control for spark max in sim!")
            else -> println("Incompatible with spark max sim!")
        }
    }

    val busVoltage: Double
        get() = motorSim.inputVoltage

    val appliedOutput: Double
        get() = motorSim.inputVoltage / 12.0

    val velocity: Double
        get() = motorSim.angularVelocityRPM * conversionFactor

    val position: Double
        get() = motorSim.angularPositionRotations * conversionFactor

    val outputCurrent: Double
        get() = motorSim.currentDrawAmps
}
