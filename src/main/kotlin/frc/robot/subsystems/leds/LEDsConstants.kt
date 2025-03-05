package frc.robot.subsystems.leds

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color

val SCROLLING_SPEED_RAINBOW: LinearVelocity = Units.MetersPerSecond.of(0.3)
val SCROLLING_SPEED_TEAM_PATTERN: LinearVelocity = Units.MetersPerSecond.of(0.2)

val BLINKING_OFF_TIME: Time = Units.Second.of(0.5)
val BLINKING_ON_TIME: Time = Units.Second.of(1.0)
val LED_SPACING: Distance = Meters.of(1 / 120.0)
const val STRIP_LENGTH = 41
const val LED_STRIP_PORT = 1
val GRIPPER_PATTERN_BRIGHTNESS: Dimensionless = Units.Percent.of(50.0)
val CLIMBER_PATTERN_BRIGHTNESS: Dimensionless = Units.Percent.of(50.0)

val BLUE_BRIGHTNESS: Dimensionless = Units.Percent.of(20.0)
val RED_BRIGHTNESS: Dimensionless = Units.Percent.of(40.0)
val ALIGN_BRIGHTNESS: Dimensionless = Units.Percent.of(40.0)
val GRADIENT_PINK: Color = Color(255, 0, 148)
val blueTeamPattern: LEDPattern =
    LEDPattern.gradient(
            LEDPattern.GradientType.kDiscontinuous,
            Color.kAqua,
            Color.kBlue
        )
        .atBrightness(BLUE_BRIGHTNESS)
        .scrollAtAbsoluteSpeed(SCROLLING_SPEED_TEAM_PATTERN, LED_SPACING)

val redTeamPattern: LEDPattern =
    LEDPattern.gradient(
            LEDPattern.GradientType.kDiscontinuous,
            Color.kRed,
            GRADIENT_PINK
        )
        .atBrightness(RED_BRIGHTNESS)
        .scrollAtAbsoluteSpeed(SCROLLING_SPEED_TEAM_PATTERN, LED_SPACING)

val alignPattern: LEDPattern = LEDPattern.solid(Color.kGreen).atBrightness(ALIGN_BRIGHTNESS)

val climbPattern: LEDPattern = LEDPattern.rainbow(255, 128)
    .scrollAtAbsoluteSpeed(
        SCROLLING_SPEED_RAINBOW,
        LED_SPACING
    )
    .atBrightness(CLIMBER_PATTERN_BRIGHTNESS)

val gripperPattern: LEDPattern = LEDPattern.solid(Color.kWhiteSmoke)
    .blink(BLINKING_ON_TIME, BLINKING_OFF_TIME)
    .atBrightness(GRIPPER_PATTERN_BRIGHTNESS)

