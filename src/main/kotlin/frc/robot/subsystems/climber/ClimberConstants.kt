package frc.robot.subsystems.climber

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val GEAR_RATIO = (1.0 / 12.0) * (30.0 / 66.0) * (12.0 / 36.0)
val UNFOLDED_ANGLE: Angle = Units.Degree.of(0.0)
val FOLDED_ANGLE: Angle = Units.Degree.of(90.0)
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.003)

var MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput.apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.Clockwise_Positive
        }
        CurrentLimits.apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimitEnable = true
            StatorCurrentLimit = 160.0
            SupplyCurrentLimit = 80.0
        }
        SoftwareLimitSwitch.apply {
            ForwardSoftLimitEnable = true
            ReverseSoftLimitEnable = true
            ForwardSoftLimitThreshold = FOLDED_ANGLE.`in`(Units.Rotations)
            ReverseSoftLimitThreshold = UNFOLDED_ANGLE.`in`(Units.Rotations)
        }
    }
