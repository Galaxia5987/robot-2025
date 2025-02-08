package frc.robot.subsystems.leds

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import frc.robot.IS_RED

val SCROLLING_SPEED_RAINBOW: LinearVelocity = Units.MetersPerSecond.of(0.3)
val SCROLLING_SPEED_TEAM_PATTERN: LinearVelocity = Units.MetersPerSecond.of(0.2)

val BLINKING_TIME: Time = Units.Second.of(2.0)
val LED_SPACING: Distance = Meters.of(1 / 120.0)
const val STRIP_LENGTH = 41
const val LED_STRIP_PORT = 0

val blueBrightness: Dimensionless = Units.Percent.of(20.0)
val redBrightness: Dimensionless = Units.Percent.of(40.0)
val gradientPink: Color = Color(255, 0, 148)

val teamPattern: LEDPattern =
    when (IS_RED) {
        false ->
            LEDPattern.gradient(
                    LEDPattern.GradientType.kDiscontinuous,
                    Color.kAqua,
                    Color.kBlue
                )
                .atBrightness(blueBrightness)
                .scrollAtAbsoluteSpeed(
                    SCROLLING_SPEED_TEAM_PATTERN,
                    LED_SPACING
                )
        true ->
            LEDPattern.gradient(
                    LEDPattern.GradientType.kDiscontinuous,
                    Color.kRed,
                    gradientPink
                )
                .atBrightness(redBrightness)
                .scrollAtAbsoluteSpeed(
                    SCROLLING_SPEED_TEAM_PATTERN,
                    LED_SPACING
                )
    }

val OFF: LEDPattern = LEDPattern.kOff
