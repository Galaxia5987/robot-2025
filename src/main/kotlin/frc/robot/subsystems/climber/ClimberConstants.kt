package frc.robot.subsystems.climber

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia

const val GEAR_RATIO =
    1 / ((1.0 / 9.0) * (1.0 / 3.0) * (1.0 / 3.0) * (30.0 / 60.0))
val UNFOLDED_ANGLE: Angle = Units.Degree.of(0.0)
val FOLDED_ANGLE: Angle = Units.Degree.of(90.0)
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.003)

var MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput.apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.CounterClockwise_Positive
        }
        Feedback.apply { SensorToMechanismRatio = GEAR_RATIO }
        CurrentLimits.apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimitEnable = true
            StatorCurrentLimit = 160.0
            SupplyCurrentLimit = 80.0
        }
    }
