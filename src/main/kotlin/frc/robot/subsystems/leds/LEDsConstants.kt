package frc.robot.subsystems.leds

import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.percent
import frc.robot.lib.extensions.sec

val SCROLLING_SPEED_RAINBOW: LinearVelocity = 0.3.mps
val SCROLLING_SPEED_TEAM_PATTERN: LinearVelocity = 0.2.mps

val BLINKING_OFF_TIME: Time = 0.5.sec
val BLINKING_ON_TIME: Time = 1.0.sec
val LED_SPACING: Distance = (1 / 120.0).m
const val STRIP_LENGTH = 41
const val LED_STRIP_PORT = 1
val GRIPPER_PATTERN_BRIGHTNESS: Dimensionless = 50.0.percent
val CLIMBER_PATTERN_BRIGHTNESS: Dimensionless = 50.0.percent

val BLUE_BRIGHTNESS: Dimensionless = 20.0.percent
val RED_BRIGHTNESS: Dimensionless = 40.0.percent
val ALIGN_BRIGHTNESS: Dimensionless = 40.0.percent
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

val alignPattern: LEDPattern =
    LEDPattern.solid(Color.kGreen).atBrightness(ALIGN_BRIGHTNESS)

val climbPattern: LEDPattern =
    LEDPattern.rainbow(255, 128)
        .scrollAtAbsoluteSpeed(SCROLLING_SPEED_RAINBOW, LED_SPACING)
        .atBrightness(CLIMBER_PATTERN_BRIGHTNESS)

val gripperPattern: LEDPattern =
    LEDPattern.solid(Color.kWhiteSmoke)
        .blink(BLINKING_ON_TIME, BLINKING_OFF_TIME)
        .atBrightness(GRIPPER_PATTERN_BRIGHTNESS)
