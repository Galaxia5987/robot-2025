package frc.robot.subsystems.climber

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val UNFOLD_POWER = 1
const val FOLD_POWER = 1
val OPEN_LATCH_POSITION: Distance = Units.Centimeter.of(0.8)
val CLOSE_LATCH_POSITION: Distance = Units.Centimeter.of(0.2)
val UNFOLDED_ANGLE: Angle = Units.Degree.of(60.0)
val FOLDED_ANGLE: Angle = Units.Degree.of(30.0)
const val GEAR_RATIO = 1.0
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.0)
var DISTANCE_THRESHOLD: Distance = Units.Centimeter.of(0.4)
var LATCH_TOLERANCE: Distance = Units.Centimeter.of(0.4)
var MOTOR_CONFIG = TalonFXConfiguration().apply {
    MotorOutput.apply {
        NeutralMode = NeutralModeValue.Brake
        Inverted = InvertedValue.Clockwise_Positive
    }
    CurrentLimits.apply {
        StatorCurrentLimitEnable = false
        SupplyCurrentLimitEnable = false
    }
    Slot0.apply {
        kD = Gains.kD
        kP = Gains.kP
        kI = Gains.kI
    }
}
var Gains = selectGainsBasedOnMode(
    Gains(
        0.0,
        0.0,
    ), Gains(
        0.0,
        0.0,
    )
)
