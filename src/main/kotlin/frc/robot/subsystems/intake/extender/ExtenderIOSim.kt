package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class ExtenderIOSim : ExtenderIO {

    override val inputs = LoggedExtenderInputs()
    private val motor =
        TalonFXSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getFalcon500Foc(1),
                MASS.`in`(Units.Kilogram),
                PINION_RADIUS.`in`(Units.Meters),
                GEAR_RATIO
            ),
            1,
            GEAR_RATIO,
            1.0,
            TalonType.FALCON_FOC
        )
    private val positionControl = PositionVoltage(0.0)

    override fun setPosition(position: Distance) {}
}
