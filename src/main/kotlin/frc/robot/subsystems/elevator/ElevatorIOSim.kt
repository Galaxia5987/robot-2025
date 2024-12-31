package frc.robot.subsystems.elevator

import MOTOR_ID
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.motors.TalonFXSim
import kotlin.math.PI

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motor = TalonFX(MOTOR_ID)

    override fun setHeight(position: Distance) {
        val rotationalPosition=Units.Rotations.of(position.`in`(Units.Centimeter)/(GEAR_RATIO * FIRST_STAGE_RATIO * 2*PI* SPROCKET_RADIUS.`in`(Units.Centimeter)))
        motor.setControl(motorPosititonRequest.withPosition(rotationalPosition))
    }
    override fun setPower(percentOutput: Double) {
        super.setPower(percentOutput)
    }

    override fun updateInputs() {
        inputs.appliedVoltege=Units.Volts.of(motor.appliedVoltage)
        inputs.carriageHeight=Units.Centimeter.of(motor.position * GEAR_RATIO * FIRST_STAGE_RATIO * (SPROCKET_RADIUS.`in`(Units.Centimeter) * 2 * PI))
    }
}
