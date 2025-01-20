package frc.robot.subsystems.climber

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

val UNLOCK_VOLTAGE = Units.Volts.of(0.0)
val OPEN_LATCH_POSITION: Angle = Units.Degree.of(0.8)
val CLOSE_LATCH_POSITION: Angle = Units.Degree.of(0.2)
val UNLOCK_ANGLE = Units.Degree.of(90.0)
val LOCK_ANGLE = Units.Degree.of(10.0)
val UNFOLDED_ANGLE: Angle = Units.Degree.of(60.0)
val FOLDED_ANGLE: Angle = Units.Degree.of(30.0)
val FOLDED_TOLERANCE = Units.Degree.of(1.0)
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.0)
val DISTANCE_THRESHOLD = Units.Centimeter.of(0.4)
val LATCH_TOLERANCE: Angle = Units.Degree.of(1.0)

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
