package frc.robot.subsystems.climber

import com.ctre.phoenix6.configs.TalonFXConfiguration
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
const val STOPPER_GEAR_RATIO = (8.0 / 72.0) * (1.0 / 15.0)
val OPEN_LATCH_POSITION: Distance = Units.Millimeters.of(0.8)
val CLOSE_LATCH_POSITION: Distance = Units.Millimeters.of(0.2)
val LATCH_TOLERANCE: Distance = Units.Millimeters.of(1.0)
val LOCK_POWER = 0.0
val UNLOCK_POWER = 0.0
val UNFOLDED_ANGLE: Angle = Units.Degree.of(0.0)
val FOLDED_ANGLE: Angle = Units.Degree.of(90.0)
val FOLDED_TOLERANCE: Angle = Units.Degree.of(1.0)
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.003)
val MOMENT_OF_INERTIA_LOCK: MomentOfInertia =
    Units.KilogramSquareMeters.of(0.003)
val DISTANCE_THRESHOLD: Distance = Units.Centimeter.of(0.4)
val STOPPER_CURRENT_THRESHOLD: Current = Units.Amps.of(0.0)

var GAINS =
    selectGainsBasedOnMode(
        Gains(kP = 0.0, kI = 0.0, kD = 0.0),
        Gains(kP = 0.015, kI = 0.0, kD = 0.045)
    )
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
        Slot0.apply {
            kP = GAINS.kP
            kI = GAINS.kI
            kD = GAINS.kD
        }
    }
